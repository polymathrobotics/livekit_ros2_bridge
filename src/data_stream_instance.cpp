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

#include "data_stream_instance.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <utility>

#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "subscription_qos.hpp"
#include "subscription_registry.hpp"
#include "utils/log_event.hpp"
#include "utils/quiesce_gate.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kSubscriptionDepth = 2U;
constexpr auto kDeliveryFailureLogThrottle = std::chrono::seconds(5);
const auto kDataStreamInstanceLogger = rclcpp::get_logger("data_stream_instance");

// Keep the topic-to-track mapping deterministic so reconnect-driven republishes reuse the same
// externally visible LiveKit identity for this ROS topic.
std::string makeDataTrackName(const std::string & topic)
{
  std::string track_name = "ros.data";
  track_name.reserve(track_name.size() + topic.size());
  for (char ch : topic) {
    track_name.push_back(ch == '/' ? '.' : ch);
  }
  return track_name;
}

}  // namespace

DataStreamInstance::DataStreamInstance(
  std::string topic,
  std::string interface_type,
  rclcpp::Node & node,
  RoomConnection & room_connection,
  SubscriptionRegistry & registry,
  QuiesceGate & callback_gate,
  const SubscriptionQosConfig * qos_config)
: node_(node)
, topic_(std::move(topic))
, interface_type_(std::move(interface_type))
, track_name_(makeDataTrackName(topic_))
, publisher_(room_connection, track_name_, node_.get_clock())
, callback_generation_(callback_gate.currentGeneration())
, registry_(registry)
, callback_gate_(callback_gate)
, qos_config_(qos_config)
{}

DataStreamInstance::~DataStreamInstance()
{
  shutdown();
}

const std::string & DataStreamInstance::trackName() const
{
  return track_name_;
}

int DataStreamInstance::suppressionIntervalMs() const
{
  return suppression_window_.intervalMs();
}

DataStreamInstance::State DataStreamInstance::state() const
{
  return publication_.current();
}

void DataStreamInstance::setSuppressionIntervalMs(int interval_ms)
{
  suppression_window_.setIntervalMs(interval_ms);
}

void DataStreamInstance::start(const std::string & requester_identity, std::size_t generation)
{
  if (!publication_.canStart()) {
    return;
  }

  // Enter pending before asking DataTrackPublisher to publish. ROS messages can still arrive
  // while the track handshake is in flight, and forwardMessage() must drop them until the
  // matching completePublish() confirms this exact generation.
  publication_.beginPublish(generation);
  LogEvent(kDataStreamInstanceLogger, "data_track_pending")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .field("requester_identity", requester_identity)
    .info();
  publisher_.publish(
    generation,
    [this](std::size_t generation) { return registry_.onDataTrackPublished(track_name_, generation); },
    [this]() { registry_.onDataTrackFailed(track_name_); });
}

void DataStreamInstance::republish(const std::string & requester_identity, std::size_t generation)
{
  if (!publication_.canRepublish()) {
    return;
  }

  resetPublication();
  // Enter pending before asking DataTrackPublisher to publish. ROS messages can still arrive
  // while the track handshake is in flight, and forwardMessage() must drop them until the
  // matching completePublish() confirms this exact generation.
  publication_.beginPublish(generation);
  LogEvent(kDataStreamInstanceLogger, "data_track_pending")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .field("requester_identity", requester_identity)
    .info();
  publisher_.publish(
    generation,
    [this](std::size_t generation) { return registry_.onDataTrackPublished(track_name_, generation); },
    [this]() { registry_.onDataTrackFailed(track_name_); });
}

bool DataStreamInstance::completePublish(std::size_t generation)
{
  if (!publication_.completePublish(generation)) {
    return false;
  }

  LogEvent(kDataStreamInstanceLogger, "data_track_published")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .info();
  return true;
}

void DataStreamInstance::failPublish()
{
  LogEvent(kDataStreamInstanceLogger, "data_track_publish_failed")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .warn();
  publication_.failPublish();
}

void DataStreamInstance::shutdown()
{
  resetPublication();
  subscription_.reset();
}

void DataStreamInstance::resetPublication()
{
  publisher_.unpublish();
  publication_.reset();
  suppression_window_.reset();
}

void DataStreamInstance::subscribe()
{
  const rclcpp::QoS base_qos(kSubscriptionDepth);
  const ResolvedSubscriptionQos qos = resolveSubscriptionQos(node_, topic_, base_qos, qos_config_);

  LogEvent(kDataStreamInstanceLogger, "subscription_qos_resolved")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("delivery", protocol::kDeliveryKindData)
    .field("interface_type", interface_type_)
    .field("source", subscriptionQosSourceString(qos.source))
    .field("reliability", subscriptionQosReliabilityString(qos.qos.reliability()))
    .field("durability", subscriptionQosDurabilityString(qos.qos.durability()))
    .field("used_publisher_qos", qos.used_publisher_qos)
    .field("mixed_reliability", qos.mixed_reliability)
    .field("mixed_durability", qos.mixed_durability)
    .field("override_id", qos.override_id)
    .field("override_pattern", qos.override_pattern)
    .info();

  // ROS may already have queued a callback when SubscriptionRegistry starts reset/shutdown.
  // The gate rejects old-session callbacks before they touch shared state, and the weak pointer
  // keeps a late callback from extending the instance lifetime past teardown.
  const std::weak_ptr<DataStreamInstance> weak_self = weak_from_this();
  subscription_ = node_.create_generic_subscription(
    topic_,
    interface_type_,
    qos.qos,
    [weak_self, callback_generation = callback_generation_, &callback_gate = callback_gate_](
      std::shared_ptr<rclcpp::SerializedMessage> message) {
      if (message == nullptr) {
        return;
      }
      if (!callback_gate.tryEnter(callback_generation)) {
        return;
      }

      ScopeExit leave_gate([&callback_gate]() { callback_gate.leave(); });
      const auto self = weak_self.lock();
      if (!self) {
        return;
      }

      self->forwardMessage(*message);
    });
}

void DataStreamInstance::forwardMessage(const rclcpp::SerializedMessage & message)
{
  // ROS callbacks may still arrive while publish is pending or after teardown wins a race. Drop
  // instead of buffering so a later publication never replays stale pre-publish data.
  if (!publication_.isPublished()) {
    return;
  }

  if (!suppression_window_.allow(Clock::now())) {
    return;
  }

  const auto & cdr = message.get_rcl_serialized_message();
  try {
    publisher_.write(cdr.buffer, cdr.buffer_length);
  } catch (const std::exception & exc) {
    LogEvent(kDataStreamInstanceLogger, "data_track_delivery_failed")
      .field("resource", topic_)
      .field("kind", "topic")
      .field("track_name", track_name_)
      .field("error", exc.what())
      .warnThrottle(*node_.get_clock(), kDeliveryFailureLogThrottle);
  }
}

}  // namespace livekit_ros2_bridge
