// Copyright (c) 2025-present Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "data_track_publisher.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/data_track_frame.h"
#include "livekit/result.h"
#include "protocol/constants.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "room_connection.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kSubscriptionDepth = 2U;
constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("data_track_publisher");

std::string makeTrackName(const std::string & topic)
{
  std::string name = "lkros.data";
  name.reserve(name.size() + topic.size());
  for (char ch : topic) {
    name.push_back(ch == '/' ? '.' : ch);
  }
  return name;
}

}  // namespace

class DataTrackPublisher::Publication final
{
public:
  using Clock = std::chrono::steady_clock;

  struct Throttle
  {
    bool allows(Clock::time_point now)
    {
      if (interval_ms == 0) {
        return true;
      }

      if (!last_delivery_at) {
        last_delivery_at = now;
        return true;
      }

      if (now - *last_delivery_at < std::chrono::milliseconds(interval_ms)) {
        return false;
      }

      last_delivery_at = now;
      return true;
    }

    int interval_ms = 0;
    std::optional<Clock::time_point> last_delivery_at;
  };

  // Shared state touched by the subscription callback. Holding this separately
  // from Publication lets the callback capture a weak_ptr and self-reject once
  // Publication has closed the gate during teardown.
  class State final
  {
  public:
    State(
      std::string topic,
      std::string track_name,
      int interval_ms,
      rclcpp::Clock::SharedPtr clock,
      RoomConnection & room_connection)
    : clock_(std::move(clock))
    , room_connection_(room_connection)
    , topic_(std::move(topic))
    , track_name_(std::move(track_name))
    , track_(room_connection_.publishDataTrack(track_name_))
    {
      if (track_ == nullptr) {
        throw std::runtime_error("LiveKit returned a null data track.");
      }
      throttle_.interval_ms = interval_ms;
    }

    State(const State &) = delete;
    State & operator=(const State &) = delete;
    State(State &&) = delete;
    State & operator=(State &&) = delete;

    void closeAndWait()
    {
      std::unique_lock<std::mutex> lock(lifecycle_mutex_);
      is_closed_ = true;
      idle_.wait(lock, [this]() { return active_callbacks_ == 0U; });
    }

    bool tryEnter()
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      if (is_closed_) {
        return false;
      }

      ++active_callbacks_;
      return true;
    }

    void leave()
    {
      {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (active_callbacks_ == 0U) {
          return;
        }
        --active_callbacks_;
      }

      idle_.notify_all();
    }

    void setIntervalMs(int interval_ms)
    {
      std::lock_guard<std::mutex> lock(throttle_mutex_);
      throttle_.interval_ms = interval_ms;
    }

    void pushMessage(const rclcpp::SerializedMessage & message)
    {
      {
        std::lock_guard<std::mutex> lock(throttle_mutex_);
        if (!throttle_.allows(Clock::now())) {
          return;
        }
      }

      const auto & cdr = message.get_rcl_serialized_message();
      const auto result = room_connection_.tryPushDataTrack(
        track_, livekit::DataTrackFrame{std::vector<std::uint8_t>(cdr.buffer, cdr.buffer + message.size())});
      if (result) {
        return;
      }

      const auto & error = result.error();
      if (error.code == livekit::LocalDataTrackTryPushErrorCode::QUEUE_FULL) {
        LogEvent(kLogger, "data_track_delivery_dropped")
          .field("resource", topic_)
          .field("track_name", track_name_)
          .field("reason", "queue_full")
          .warnThrottle(*clock_, kLogThrottle);
        return;
      }

      LogEvent(kLogger, "data_track_push_failed")
        .field("resource", topic_)
        .field("track_name", track_name_)
        .field("sdk_error_code", static_cast<std::uint32_t>(error.code))
        .fieldOr("error", error.message)
        .warnThrottle(*clock_, kLogThrottle);
    }

    void unpublishTrack()
    {
      if (track_ == nullptr) {
        return;
      }

      try {
        room_connection_.unpublishDataTrack(track_);
        track_.reset();
      } catch (...) {
        LogEvent(kLogger, "data_track_unpublish_failed")
          .field("track_name", track_name_)
          .fieldException("error", std::current_exception())
          .warn();
      }
    }

  private:
    rclcpp::Clock::SharedPtr clock_;
    RoomConnection & room_connection_;
    std::string topic_;
    std::string track_name_;
    std::shared_ptr<livekit::LocalDataTrack> track_;

    std::mutex lifecycle_mutex_;
    std::condition_variable idle_;
    bool is_closed_ = false;
    std::size_t active_callbacks_ = 0U;

    std::mutex throttle_mutex_;
    Throttle throttle_;
  };

  Publication(
    std::string topic,
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
  , topic_(std::move(topic))
  , interface_type_(std::move(interface_type))
  , state_(std::make_shared<State>(topic_, std::move(track_name), interval_ms, std::move(clock), room_connection))
  {
    try {
      subscribe();
    } catch (...) {
      state_->unpublishTrack();
      throw;
    }
  }

  ~Publication()
  {
    state_->closeAndWait();
    subscription_.reset();
    state_->unpublishTrack();
  }

  Publication(const Publication &) = delete;
  Publication & operator=(const Publication &) = delete;
  Publication(Publication &&) = delete;
  Publication & operator=(Publication &&) = delete;

  void setIntervalMs(int interval_ms)
  {
    state_->setIntervalMs(interval_ms);
  }

private:
  void subscribe()
  {
    const rclcpp::QoS base_qos{rclcpp::KeepLast(kSubscriptionDepth)};
    const ResolvedSubscriptionQos qos = resolveSubscriptionQos(graph_, topic_, base_qos, qos_config_);

    LogEvent(kLogger, "subscription_qos_resolved")
      .field("resource", topic_)
      .field("delivery", protocol::kDataDeliveryKind)
      .field("interface_type", interface_type_)
      .field("publisher_count", qos.publisher_count)
      .field("source", subscriptionQosSourceString(qos.source))
      .field("reliability", subscriptionQosReliabilityString(qos.qos.reliability()))
      .field("durability", subscriptionQosDurabilityString(qos.qos.durability()))
      .fieldIf(qos.used_publisher_qos, "used_publisher_qos", true)
      .fieldIf(qos.mixed_reliability, "mixed_reliability", true)
      .fieldIf(qos.mixed_durability, "mixed_durability", true)
      .fieldIfNotEmpty("override_id", qos.override_id)
      .fieldIfNotEmpty("override_pattern", qos.override_pattern)
      .info();

    subscription_ = rclcpp::create_generic_subscription(
      topics_,
      topic_,
      interface_type_,
      qos.qos,
      [weak_state = std::weak_ptr<State>(state_)](std::shared_ptr<rclcpp::SerializedMessage> message) {
        const auto state = weak_state.lock();
        if (state == nullptr || !state->tryEnter()) {
          return;
        }
        ScopeExit leave_state([&state]() { state->leave(); });
        state->pushMessage(*message);
      });
  }

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  const SubscriptionQosConfig * qos_config_;

  std::string topic_;
  std::string interface_type_;

  std::shared_ptr<State> state_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
};

std::shared_ptr<DataTrackPublisher> DataTrackPublisher::create(
  std::string topic,
  std::string interface_type,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  RoomConnection & room_connection,
  const SubscriptionQosConfig * qos_config)
{
  return std::shared_ptr<DataTrackPublisher>(new DataTrackPublisher(
    std::move(topic),
    std::move(interface_type),
    std::move(topics),
    std::move(graph),
    std::move(clock),
    room_connection,
    qos_config));
}

DataTrackPublisher::DataTrackPublisher(
  std::string topic,
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
, topic_(std::move(topic))
, interface_type_(std::move(interface_type))
, track_name_(makeTrackName(topic_))
{}

DataTrackPublisher::~DataTrackPublisher() = default;

void DataTrackPublisher::publish()
{
  if (publication_ != nullptr) {
    return;
  }

  if (publish_failed_) {
    LogEvent(kLogger, "data_track_pending")
      .field("resource", topic_)
      .field("track_name", track_name_)
      .field("reason", "retry_after_publish_failure")
      .info();
  }

  try {
    publication_ = std::make_unique<Publication>(
      topic_, interface_type_, track_name_, interval_ms_, topics_, graph_, clock_, room_connection_, qos_config_);
    publish_failed_ = false;
    LogEvent(kLogger, "data_track_published").field("resource", topic_).field("track_name", track_name_).info();
  } catch (...) {
    publish_failed_ = true;
    LogEvent(kLogger, "data_track_publish_error")
      .field("resource", topic_)
      .field("track_name", track_name_)
      .field("stage", "activate_publication")
      .fieldException("error", std::current_exception())
      .warn();
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

void DataTrackPublisher::republish()
{
  publication_.reset();
  publish();
}

void DataTrackPublisher::setIntervalMs(int interval_ms)
{
  interval_ms_ = interval_ms;
  if (publication_ == nullptr) {
    return;
  }

  publication_->setIntervalMs(interval_ms);
}

const std::string & DataTrackPublisher::name() const
{
  return track_name_;
}

}  // namespace livekit_ros2_bridge
