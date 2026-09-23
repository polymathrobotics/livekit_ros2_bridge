// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/data_track_publisher.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/room_connection.hpp"
#include "core/subscription_qos.hpp"
#include "livekit/data_track_error.h"
#include "livekit/data_track_frame.h"
#include "livekit/result.h"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "utils/callback_gate.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kSubscriptionDepth = 2U;
constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("data_track_publisher");

std::string makeTrackName(const std::string & ros_topic)
{
  std::string name = "lkros.data";
  name.reserve(name.size() + ros_topic.size());
  for (char ch : ros_topic) {
    name.push_back(ch == '/' ? '.' : ch);
  }
  return name;
}

}  // namespace

class DataTrackPublisher::Publication final
{
public:
  // Gates callbacks and keeps the LiveKit track alive while ROS callbacks drain.
  class State final
  {
  public:
    State(
      std::string ros_topic,
      std::string track_name,
      int interval_ms,
      rclcpp::Clock::SharedPtr clock,
      RoomConnection & room_connection)
    : clock_(std::move(clock))
    , room_connection_(room_connection)
    , ros_topic_(std::move(ros_topic))
    , track_name_(std::move(track_name))
    , track_(room_connection_.publishDataTrack(track_name_))
    {
      if (track_ == nullptr) {
        throw std::runtime_error("LiveKit returned a null data track.");
      }
      interval_ms_ = interval_ms;
    }

    State(const State &) = delete;
    State & operator=(const State &) = delete;
    State(State &&) = delete;
    State & operator=(State &&) = delete;

    CallbackGate callback_gate_;

    void setTransientLocal(bool transient_local)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_transient_local_ = transient_local;
    }

    void setIntervalMs(int interval_ms)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      interval_ms_ = interval_ms;
    }

    bool isTransientLocal() const
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      return is_transient_local_;
    }

    std::shared_ptr<const std::vector<std::uint8_t>> cachedCdr() const
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      return cached_cdr_;
    }

    void push(const rclcpp::SerializedMessage & message)
    {
      const auto & cdr = message.get_rcl_serialized_message();

      {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (is_transient_local_) {
          // Build the buffer once and share it immutably; echo-once reads and the byte-stream
          // send then alias this buffer instead of deep-copying the (possibly large) CDR.
          cached_cdr_ = std::make_shared<const std::vector<std::uint8_t>>(cdr.buffer, cdr.buffer + message.size());
        }

        if (interval_ms_ != 0) {
          const auto now = std::chrono::steady_clock::now();
          if (!last_push_at_) {
            last_push_at_ = now;
          } else if (now - *last_push_at_ < std::chrono::milliseconds(interval_ms_)) {
            return;
          } else {
            last_push_at_ = now;
          }
        }
      }

      const auto result = room_connection_.tryPushDataTrack(
        track_, livekit::DataTrackFrame{std::vector<std::uint8_t>(cdr.buffer, cdr.buffer + message.size())});
      if (result) {
        return;
      }

      const auto & error = result.error();
      LogEvent(kLogger, "data_track_push_failed")
        .field("resource", ros_topic_)
        .field("track_name", track_name_)
        .fieldEnum("sdk_error_code", error.code)
        .fieldOr("error", error.message)
        .warnThrottle(*clock_, kLogThrottle);
    }

    void unpublish()
    {
      if (track_ == nullptr) {
        return;
      }

      try {
        room_connection_.unpublishDataTrack(track_);
        track_.reset();
      } catch (...) {
        // RoomConnection logs SDK unpublish failures before propagating them.
      }
    }

  private:
    rclcpp::Clock::SharedPtr clock_;
    RoomConnection & room_connection_;
    std::string ros_topic_;
    std::string track_name_;
    std::shared_ptr<livekit::LocalDataTrack> track_;

    mutable std::mutex state_mutex_;
    int interval_ms_ = 0;
    std::optional<std::chrono::steady_clock::time_point> last_push_at_;
    bool is_transient_local_ = false;
    std::shared_ptr<const std::vector<std::uint8_t>> cached_cdr_;
  };

  Publication(
    std::string ros_topic,
    std::string interface_type,
    std::string track_name,
    int interval_ms,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    RoomConnection & room_connection,
    const SubscriptionQosConfig * qos_config)
  : topics_(std::move(topics))
  , graph_(std::move(graph))
  , qos_config_(qos_config)
  , ros_topic_(std::move(ros_topic))
  , interface_type_(std::move(interface_type))
  , track_name_(std::move(track_name))
  , state_(std::make_shared<State>(ros_topic_, track_name_, interval_ms, std::move(clock), room_connection))
  {
    try {
      subscribe();
    } catch (...) {
      LogEvent(kLogger, "data_track_publish_failed")
        .field("resource", ros_topic_)
        .field("interface_type", interface_type_)
        .field("track_name", track_name_)
        .fieldException("error", std::current_exception())
        .warn();
      state_->unpublish();
      throw;
    }
  }

  ~Publication()
  {
    (void)state_->callback_gate_.closeAndWait();
    subscription_.reset();
    state_->unpublish();
  }

  Publication(const Publication &) = delete;
  Publication & operator=(const Publication &) = delete;
  Publication(Publication &&) = delete;
  Publication & operator=(Publication &&) = delete;

  void setIntervalMs(int interval_ms)
  {
    state_->setIntervalMs(interval_ms);
  }

  bool isTransientLocal() const
  {
    return state_->isTransientLocal();
  }

  std::shared_ptr<const std::vector<std::uint8_t>> cachedCdr() const
  {
    return state_->cachedCdr();
  }

private:
  void subscribe()
  {
    const rclcpp::QoS base_qos{rclcpp::KeepLast(kSubscriptionDepth)};
    const ResolvedSubscriptionQos qos = resolveSubscriptionQos(graph_, ros_topic_, base_qos, qos_config_);

    if (qos.source != SubscriptionQosResolutionSource::Fallback || qos.mixed_reliability || qos.mixed_durability) {
      LogEvent(kLogger, "subscription_qos_resolved")
        .field("resource", ros_topic_)
        .field("interface_type", interface_type_)
        .field("publisher_count", qos.publisher_count)
        .fieldEnum("source", qos.source)
        .fieldEnum("reliability", qos.qos.reliability())
        .fieldEnum("durability", qos.qos.durability())
        .fieldIf(qos.mixed_reliability, "mixed_reliability", true)
        .fieldIf(qos.mixed_durability, "mixed_durability", true)
        .fieldIfNotEmpty("override_id", qos.override_id)
        .info();
    }

    state_->setTransientLocal(qos.qos.durability() == rclcpp::DurabilityPolicy::TransientLocal);

    subscription_ = rclcpp::create_generic_subscription(
      topics_,
      ros_topic_,
      interface_type_,
      qos.qos,
      [weak_state = std::weak_ptr<State>(state_)](std::shared_ptr<rclcpp::SerializedMessage> message) {
        const auto state = weak_state.lock();
        if (state == nullptr) {
          return;
        }
        (void)state->callback_gate_.run([&state, &message]() { state->push(*message); });
      });
  }

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  const SubscriptionQosConfig * qos_config_;

  std::string ros_topic_;
  std::string interface_type_;
  std::string track_name_;

  std::shared_ptr<State> state_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
};

DataTrackPublisher::DataTrackPublisher(
  std::string ros_topic,
  std::string interface_type,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  RoomConnection & room_connection,
  const SubscriptionQosConfig * qos_config)
: topics_(std::move(topics))
, graph_(std::move(graph))
, clock_(std::move(clock))
, room_connection_(room_connection)
, qos_config_(qos_config)
, ros_topic_(std::move(ros_topic))
, interface_type_(std::move(interface_type))
, track_name_(makeTrackName(ros_topic_))
{}

DataTrackPublisher::~DataTrackPublisher() = default;

void DataTrackPublisher::publish()
{
  if (publication_ != nullptr) {
    return;
  }

  try {
    publication_ = std::make_unique<Publication>(
      ros_topic_, interface_type_, track_name_, interval_ms_, topics_, graph_, clock_, room_connection_, qos_config_);
  } catch (...) {
    // Publish failures are logged at their owning boundary; later publish() calls retry.
  }
}

int DataTrackPublisher::intervalMs() const
{
  return interval_ms_;
}

bool DataTrackPublisher::isPublished() const
{
  return publication_ != nullptr;
}

SubscriptionQos DataTrackPublisher::qos() const
{
  const bool is_transient_local = publication_ != nullptr && publication_->isTransientLocal();
  return SubscriptionQos{is_transient_local ? "transient_local" : "volatile"};
}

void DataTrackPublisher::setIntervalMs(int interval_ms)
{
  interval_ms_ = interval_ms;
  if (publication_ == nullptr) {
    return;
  }

  publication_->setIntervalMs(interval_ms);
}

const std::string & DataTrackPublisher::trackName() const
{
  return track_name_;
}

std::optional<CachedMessage> DataTrackPublisher::cachedMessage() const
{
  if (publication_ == nullptr) {
    return std::nullopt;
  }
  auto cdr = publication_->cachedCdr();
  if (cdr == nullptr) {
    return std::nullopt;
  }
  return CachedMessage{ros_topic_, std::move(cdr)};
}

}  // namespace livekit_ros2_bridge
