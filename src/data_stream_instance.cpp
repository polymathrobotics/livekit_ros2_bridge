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

constexpr std::size_t kDataSubscriptionDepth = 2U;
constexpr auto kTrackDeliveryFailureLogThrottlePeriod = std::chrono::seconds(5);
const auto kDataStreamInstanceLogger = rclcpp::get_logger("data_stream_instance");

}  // namespace

std::shared_ptr<DataStreamInstance> DataStreamInstance::create(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  SubscriptionRegistry & subscription_registry,
  std::string topic,
  std::string interface_type,
  int applied_interval_ms,
  std::size_t publish_generation,
  QuiesceGate & message_callback_gate,
  const SubscriptionQosConfig * subscription_qos_config)
{
  auto instance = std::shared_ptr<DataStreamInstance>(new DataStreamInstance(
    node,
    room_connection,
    subscription_registry,
    std::move(topic),
    std::move(interface_type),
    applied_interval_ms,
    publish_generation,
    message_callback_gate,
    subscription_qos_config));
  instance->initializeSubscription();
  return instance;
}

DataStreamInstance::DataStreamInstance(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  SubscriptionRegistry & subscription_registry,
  std::string topic,
  std::string interface_type,
  int applied_interval_ms,
  std::size_t publish_generation,
  QuiesceGate & message_callback_gate,
  const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, topic_(std::move(topic))
, interface_type_(std::move(interface_type))
, track_name_(deriveTrackName(topic_))
, data_track_publisher_(room_connection, track_name_, node_.get_clock())
, applied_interval_ms_(applied_interval_ms)
, generation_(publish_generation)
, callback_generation_(message_callback_gate.currentGeneration())
, subscription_registry_(subscription_registry)
, message_callback_gate_(message_callback_gate)
, subscription_qos_config_(subscription_qos_config)
{}

const std::string & DataStreamInstance::trackName() const
{
  return track_name_;
}

int DataStreamInstance::appliedIntervalMs() const
{
  return applied_interval_ms_;
}

DataStreamInstance::State DataStreamInstance::state() const
{
  return state_;
}

void DataStreamInstance::updateAppliedIntervalMs(int applied_interval_ms)
{
  applied_interval_ms_ = applied_interval_ms;
}

void DataStreamInstance::start(const std::string & requester_identity)
{
  if (state_ != State::kNone && state_ != State::kFailed) {
    return;
  }

  publishPendingDataTrack(requester_identity);
}

void DataStreamInstance::republish(const std::string & requester_identity)
{
  if (state_ != State::kPublished) {
    return;
  }

  data_track_publisher_.unpublish();
  state_ = State::kNone;
  last_sent_time_.reset();
  publishPendingDataTrack(requester_identity);
}

bool DataStreamInstance::onPublishComplete(std::size_t generation)
{
  if (state_ != State::kPending) {
    return false;
  }
  if (generation_ != generation) {
    return false;
  }

  state_ = State::kPublished;
  LogEvent(kDataStreamInstanceLogger, "data_track_published")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .info();
  return true;
}

void DataStreamInstance::onPublishFailed()
{
  LogEvent(kDataStreamInstanceLogger, "data_track_publish_failed")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .warn();
  state_ = State::kFailed;
}

void DataStreamInstance::shutdown()
{
  data_track_publisher_.shutdown();
  state_ = State::kNone;
  last_sent_time_.reset();
  subscription_handle_.reset();
}

void DataStreamInstance::initializeSubscription()
{
  const rclcpp::QoS base_qos(kDataSubscriptionDepth);
  const ResolvedSubscriptionQos resolved_qos =
    resolveTopicSubscriptionQos(node_, topic_, base_qos, subscription_qos_config_);

  LogEvent(kDataStreamInstanceLogger, "subscription_qos_resolved")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("delivery", protocol::kDeliveryKindData)
    .field("interface_type", interface_type_)
    .field("source", subscriptionQosResolutionSourceToString(resolved_qos.source))
    .field("reliability", reliabilityPolicyToString(resolved_qos.qos.reliability()))
    .field("durability", durabilityPolicyToString(resolved_qos.qos.durability()))
    .field("used_publisher_qos", resolved_qos.used_publisher_qos)
    .field("mixed_reliability", resolved_qos.mixed_reliability)
    .field("mixed_durability", resolved_qos.mixed_durability)
    .field("override_id", resolved_qos.matched_override_id)
    .field("override_pattern", resolved_qos.matched_override_pattern)
    .info();

  const std::weak_ptr<DataStreamInstance> weak_self = weak_from_this();
  subscription_handle_ = node_.create_generic_subscription(
    topic_,
    interface_type_,
    resolved_qos.qos,
    [weak_self, callback_generation = callback_generation_, &message_callback_gate = message_callback_gate_](
      std::shared_ptr<rclcpp::SerializedMessage> message) {
      if (message == nullptr) {
        return;
      }
      if (!message_callback_gate.tryEnter(callback_generation)) {
        return;
      }

      ScopeExit finish_delivery([&message_callback_gate]() { message_callback_gate.leave(); });
      const auto self = weak_self.lock();
      if (!self) {
        return;
      }

      self->handleSerializedMessage(*message);
    });
}

void DataStreamInstance::handleSerializedMessage(const rclcpp::SerializedMessage & message)
{
  if (shouldSkipDueToInterval()) {
    return;
  }

  if (state_ != State::kPending && state_ != State::kPublished) {
    return;
  }

  const auto & rcl_msg = message.get_rcl_serialized_message();
  try {
    data_track_publisher_.tryPush(rcl_msg.buffer, rcl_msg.buffer_length);
  } catch (const std::exception & exc) {
    LogEvent(kDataStreamInstanceLogger, "data_track_delivery_failed")
      .field("resource", topic_)
      .field("kind", "topic")
      .field("track_name", track_name_)
      .field("error", exc.what())
      .warnThrottle(*node_.get_clock(), kTrackDeliveryFailureLogThrottlePeriod);
  }
}

bool DataStreamInstance::shouldSkipDueToInterval()
{
  if (applied_interval_ms_ == 0) {
    return false;
  }

  const auto now = Clock::now();
  const auto suppression_window = std::chrono::milliseconds(applied_interval_ms_);
  const bool within_suppression_window = last_sent_time_ && now - *last_sent_time_ < suppression_window;
  if (within_suppression_window) {
    return true;
  }

  last_sent_time_ = now;
  return false;
}

void DataStreamInstance::publishPendingDataTrack(const std::string & requester_identity)
{
  state_ = State::kPending;
  LogEvent(kDataStreamInstanceLogger, "data_track_pending")
    .field("resource", topic_)
    .field("kind", "topic")
    .field("track_name", track_name_)
    .field("requester_identity", requester_identity)
    .info();
  data_track_publisher_.publish(
    generation_,
    [this](std::size_t generation) { return subscription_registry_.onDataTrackPublished(track_name_, generation); },
    [this]() { subscription_registry_.onDataTrackFailed(track_name_); });
}

std::string DataStreamInstance::deriveTrackName(const std::string & normalized_topic)
{
  std::string name = "ros.data";
  for (char ch : normalized_topic) {
    name.push_back(ch == '/' ? '.' : ch);
  }
  return name;
}

}  // namespace livekit_ros2_bridge
