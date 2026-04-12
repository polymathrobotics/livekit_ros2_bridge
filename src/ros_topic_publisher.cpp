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

#include "ros_topic_publisher.hpp"

#include <chrono>
#include <cstring>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "utils/interface_types.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kPublisherDepth = 10U;
constexpr std::size_t kMaxCachedPublishers = 50U;
constexpr auto kPublishLogThrottleMs = 5000;
const auto kTopicPublisherLogger = rclcpp::get_logger("topic_publisher");

rclcpp::SerializedMessage wrapSerializedPayload(const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  if (!payload.empty()) {
    std::memcpy(rcl_msg.buffer, payload.data(), payload.size());
  }
  rcl_msg.buffer_length = payload.size();
  return serialized;
}

}  // namespace

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy)
: RosTopicPublisher(node, std::move(access_policy), kMaxCachedPublishers)
{}

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, std::size_t max_cached_publishers)
: node_(node)
, access_policy_(std::move(access_policy))
, max_cached_publishers_(max_cached_publishers)
, cached_publishers_(max_cached_publishers)
{}

void RosTopicPublisher::publish(const std::string & requester_identity, const TopicPublishCommand & command)
{
  const std::string & topic = command.topic;

  // Publish commands come from a streaming control path, so this LiveKit -> ROS
  // ingress publisher is intentionally best-effort: invalid or late commands are
  // dropped after logging.
  if (is_shutdown_.load()) {
    LogEvent(kTopicPublisherLogger, "publish_request_rejected")
      .field("reason", "shutdown")
      .field("resource", "topics")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", command.interface_type)
      .warnThrottle(*node_.get_clock(), std::chrono::milliseconds(kPublishLogThrottleMs));
    return;
  }

  if (!access_policy_.allows(AccessOperation::Publish, topic)) {
    LogEvent(kTopicPublisherLogger, "publish_request_rejected")
      .field("reason", "forbidden")
      .field("resource", "topics")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", command.interface_type)
      .warn();
    return;
  }

  std::string interface_type;
  try {
    interface_type = resolveTopicTypeOrThrow(topic, command.interface_type);
  } catch (const std::exception & exc) {
    LogEvent(kTopicPublisherLogger, "publish_request_rejected")
      .field("reason", "invalid_request")
      .field("resource", "topics")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", command.interface_type)
      .field("error", exc.what())
      .warn();
    return;
  }

  rclcpp::SerializedMessage serialized = wrapSerializedPayload(command.cdr_payload);

  try {
    publishWithResolvedPublisher(topic, interface_type, serialized);
  } catch (const std::exception & exc) {
    LogEvent(kTopicPublisherLogger, "publish_request_failed")
      .field("reason", "internal")
      .field("resource", "topics")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", interface_type)
      .field("error", exc.what())
      .error();
    return;
  }
}

void RosTopicPublisher::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }

  const std::size_t cached_publishers = cached_publishers_.size();
  LogEvent(kTopicPublisherLogger, "topic_publisher_state_changed")
    .field("reason", "shutdown")
    .field("action", "clear_cached_publishers")
    .field("cached_publishers", cached_publishers)
    .info();

  cached_publishers_.clear();
}

std::string RosTopicPublisher::resolveTopicTypeOrThrow(
  const std::string & topic, const std::string & requested_interface_type) const
{
  std::string expected_type;

  if (const auto cached_publisher = cached_publishers_.peek(topic); cached_publisher.has_value()) {
    // Reuse the cache's interface type once a publisher exists so later
    // commands stay consistent even if graph introspection lags that creation.
    expected_type = cached_publisher->interface_type;
  } else {
    expected_type = requireSingleInterfaceType(node_.get_topic_names_and_types(), topic, "topic");
  }

  if (expected_type != requested_interface_type) {
    throw std::invalid_argument("type mismatch expected=" + expected_type + " got=" + requested_interface_type);
  }
  return expected_type;
}

void RosTopicPublisher::publishWithResolvedPublisher(
  const std::string & topic, const std::string & interface_type, const rclcpp::SerializedMessage & serialized)
{
  const auto cached_publisher = cached_publishers_.peek(topic);
  const bool was_cached = cached_publisher.has_value();
  std::shared_ptr<rclcpp::GenericPublisher> resolved_publisher;
  if (!was_cached) {
    const rclcpp::QoS qos(kPublisherDepth);
    resolved_publisher = node_.create_generic_publisher(topic, interface_type, qos);
  } else {
    resolved_publisher = cached_publisher->publisher;
  }

  if (before_publish_hook_for_test_) {
    before_publish_hook_for_test_();
  }
  resolved_publisher->publish(serialized);

  // Refresh recency only after a successful publish so failed attempts do not
  // change bounded-cache residency.
  if (was_cached) {
    (void)cached_publishers_.touch(topic);
    return;
  }

  // Enforce the cap after serving the current command: the publish succeeds and
  // an older cached publisher is discarded to make room for future use.
  const auto evicted_publisher =
    cached_publishers_.insertOrAssign(topic, CachedPublisher{interface_type, std::move(resolved_publisher)});
  if (!evicted_publisher.has_value()) {
    return;
  }

  if (const std::size_t count = evicted_publisher_warning_throttle_.recordAndTakePendingCount(); count > 0U) {
    LogEvent(kTopicPublisherLogger, "publisher_cache_evicted")
      .field("reason", "max_topics_exceeded")
      .field("topic", topic)
      .field("evicted_topic", evicted_publisher->key)
      .field("count", count)
      .field("policy", "lru")
      .field("max_topics", static_cast<int>(max_cached_publishers_))
      .warn();
  }
}

void RosTopicPublisher::setBeforePublishHookForTest(std::function<void()> hook)
{
  before_publish_hook_for_test_ = std::move(hook);
}

}  // namespace livekit_ros2_bridge
