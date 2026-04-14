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

#include "data_stream_registry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "utils/quiesce_gate.hpp"
#include "utils/scope_exit.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kSubscriptionDepth = 2U;
constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("data_stream_instance");

// Keep the topic-to-track mapping deterministic so reconnect-driven republishes reuse the same
// externally visible LiveKit identity for this ROS topic.
std::string makeDataTrackName(const std::string & topic)
{
  std::string name = "ros.data";
  name.reserve(name.size() + topic.size());
  for (char ch : topic) {
    name.push_back(ch == '/' ? '.' : ch);
  }
  return name;
}

}  // namespace

DataStreamInstance::DataStreamInstance(
  std::string topic,
  std::string interface_type,
  rclcpp::Node & node,
  RoomConnection & room_connection,
  DataStreamRegistry & registry,
  QuiesceGate & callback_gate,
  const SubscriptionQosConfig * qos_config)
: node_(node)
, topic_(std::move(topic))
, interface_type_(std::move(interface_type))
, track_name_(makeDataTrackName(topic_))
, publisher_(room_connection, track_name_, node_.get_clock())
, registry_(registry)
, qos_config_(qos_config)
, gate_generation_(callback_gate.currentGeneration())
, callback_gate_(callback_gate)
{}

DataStreamInstance::~DataStreamInstance()
{
  shutdown();
}

void DataStreamInstance::setIntervalMs(int interval_ms)
{
  suppression_window_.setIntervalMs(interval_ms);
}

void DataStreamInstance::start(std::size_t generation)
{
  const State prev_state = publication_.current();
  if (!publication_.canStart()) {
    return;
  }

  // Enter pending before asking DataTrackPublisher to publish. ROS messages can still arrive
  // while the track handshake is in flight, and forwardMessage() must drop them until the
  // matching completePublish() confirms this exact generation.
  publication_.beginPublish(generation);
  if (prev_state == State::kFailed) {
    LogEvent(kLogger, "data_track_pending")
      .field("resource", topic_)
      .field("track_name", track_name_)
      .field("reason", "retry_after_publish_failure")
      .info();
  }
  publisher_.publish(
    generation,
    [this](std::size_t generation) { return registry_.onTrackPublished(track_name_, generation); },
    [this]() { registry_.onTrackFailed(track_name_); });
}

void DataStreamInstance::republish(std::size_t generation)
{
  if (!publication_.canRepublish()) {
    return;
  }

  publisher_.unpublish();
  publication_.reset();
  suppression_window_.reset();
  // Enter pending before asking DataTrackPublisher to publish. ROS messages can still arrive
  // while the track handshake is in flight, and forwardMessage() must drop them until the
  // matching completePublish() confirms this exact generation.
  publication_.beginPublish(generation);
  publisher_.publish(
    generation,
    [this](std::size_t generation) { return registry_.onTrackPublished(track_name_, generation); },
    [this]() { registry_.onTrackFailed(track_name_); });
}

bool DataStreamInstance::completePublish(std::size_t generation)
{
  if (!publication_.completePublish(generation)) {
    return false;
  }

  LogEvent(kLogger, "data_track_published").field("resource", topic_).field("track_name", track_name_).info();
  return true;
}

void DataStreamInstance::failPublish()
{
  publication_.failPublish();
}

void DataStreamInstance::shutdown()
{
  publisher_.unpublish();
  publication_.reset();
  suppression_window_.reset();
  subscription_.reset();
}

const std::string & DataStreamInstance::trackName() const
{
  return track_name_;
}

int DataStreamInstance::intervalMs() const
{
  return suppression_window_.intervalMs();
}

DataStreamInstance::State DataStreamInstance::state() const
{
  return publication_.current();
}

void DataStreamInstance::subscribe()
{
  const rclcpp::QoS base_qos(kSubscriptionDepth);
  const ResolvedSubscriptionQos qos = resolveSubscriptionQos(node_, topic_, base_qos, qos_config_);

  LogEvent(kLogger, "subscription_qos_resolved")
    .field("resource", topic_)
    .field("delivery", wire::protocol::kDeliveryKindData)
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

  // ROS may already have queued a callback when DataStreamRegistry starts reset/shutdown.
  // The gate rejects old-session callbacks before they touch shared state, and the weak pointer
  // keeps a late callback from extending the instance lifetime past teardown.
  const std::weak_ptr<DataStreamInstance> weak = weak_from_this();
  subscription_ = node_.create_generic_subscription(
    topic_,
    interface_type_,
    qos.qos,
    [weak, generation = gate_generation_, &callback_gate = callback_gate_](
      std::shared_ptr<rclcpp::SerializedMessage> message) {
      if (message == nullptr) {
        return;
      }

      if (!callback_gate.tryEnter(generation)) {
        return;
      }

      ScopeExit leave_gate([&callback_gate]() { callback_gate.leave(); });
      const auto self = weak.lock();
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
    LogEvent(kLogger, "data_track_delivery_failed")
      .field("resource", topic_)
      .field("track_name", track_name_)
      .field("error", exc.what())
      .warnThrottle(*node_.get_clock(), kLogThrottle);
  }
}

}  // namespace livekit_ros2_bridge
