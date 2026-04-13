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
#include <optional>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kPublisherDepth = 10U;
constexpr std::size_t kDefaultPublisherCacheLimit = 50U;
constexpr auto kRejectedPublishWarningThrottlePeriod = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("topic_publisher");

LogEvent & addPublishRequestFields(
  LogEvent & event,
  const std::string & topic,
  const std::string & requester_identity,
  const std::string * interface_type = nullptr)
{
  event.field("topic", topic).field("requester_identity", requester_identity);
  if (interface_type != nullptr) {
    event.field("interface_type", *interface_type);
  }
  return event;
}

}  // namespace

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy)
: RosTopicPublisher(node, std::move(access_policy), kDefaultPublisherCacheLimit)
{}

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, std::size_t cache_limit)
: node_(node)
, access_policy_(std::move(access_policy))
, cache_limit_(cache_limit)
, cache_(cache_limit)
{}

void RosTopicPublisher::publish(const std::string & requester_identity, const TopicPublishCommand & command)
{
  const std::string & topic = command.topic;

  if (is_shutdown_.load()) {
    LogEvent event(kLogger, "publish_request_rejected");
    addPublishRequestFields(event.field("reason", "shutdown"), topic, requester_identity);
    event.warnThrottle(*node_.get_clock(), kRejectedPublishWarningThrottlePeriod);
    return;
  }

  if (!access_policy_.allows(AccessOperation::Publish, topic)) {
    LogEvent event(kLogger, "publish_request_rejected");
    addPublishRequestFields(event.field("reason", "forbidden"), topic, requester_identity);
    event.warn();
    return;
  }

  std::string interface_type;
  std::shared_ptr<rclcpp::GenericPublisher> ros_publisher;
  std::optional<CachedPublisher> cached_publisher;
  try {
    cached_publisher = cache_.peek(topic);
    // Cache hits deliberately skip the ROS graph. Once a publish succeeds, the
    // cached publisher pins the interface type for that topic until eviction or
    // shutdown clears the entry.
    if (cached_publisher.has_value()) {
      interface_type = cached_publisher->interface_type;
      ros_publisher = cached_publisher->ros_publisher;
    } else {
      const auto graph_topics = topic_graph_provider_ ? topic_graph_provider_() : node_.get_topic_names_and_types();
      interface_type = requireSingleInterfaceType(graph_topics, topic, "topic");
    }

    if (interface_type != command.interface_type) {
      throw std::invalid_argument("type mismatch expected=" + interface_type + " got=" + command.interface_type);
    }
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "publish_request_rejected");
    addPublishRequestFields(
      event.field("reason", "invalid_request"), topic, requester_identity, &command.interface_type);
    event.field("error", exc.what()).warn();
    return;
  }

  // Control packets already carry a CDR payload, so copy the bytes directly
  // into SerializedMessage without a deserialize/serialize round trip.
  rclcpp::SerializedMessage serialized(command.cdr.size());
  auto & rcl_message = serialized.get_rcl_serialized_message();
  if (!command.cdr.empty()) {
    std::memcpy(rcl_message.buffer, command.cdr.data(), command.cdr.size());
  }
  rcl_message.buffer_length = command.cdr.size();

  try {
    if (!ros_publisher) {
      const rclcpp::QoS qos(kPublisherDepth);
      ros_publisher = node_.create_generic_publisher(topic, interface_type, qos);
    }

    if (before_publish_handler_) {
      before_publish_handler_();
    }
    // before_publish_handler_ can trigger shutdown after publisher creation, so
    // recheck before sending bytes.
    if (is_shutdown_.load()) {
      LogEvent event(kLogger, "publish_request_dropped");
      addPublishRequestFields(event.field("reason", "shutdown"), topic, requester_identity);
      event.warn();
      return;
    }

    ros_publisher->publish(serialized);
    // If shutdown is already visible here, bail out before touching cache state
    // that teardown is intentionally trying to drop.
    if (is_shutdown_.load()) {
      return;
    }

    // Refresh recency only after a successful publish. If an in-flight cached
    // publisher was evicted meanwhile, reinsert the handle that actually
    // published so later commands can still reuse it.
    if (cached_publisher.has_value() && cache_.touch(topic)) {
      return;
    }

    // Enforce the cap only after the current publish succeeds.
    const auto evicted = cache_.insertOrAssign(topic, CachedPublisher{interface_type, std::move(ros_publisher)});
    if (!evicted.has_value()) {
      return;
    }

    const std::size_t count = eviction_warning_throttle_.recordAndTakePendingCount();
    if (count == 0U) {
      return;
    }

    LogEvent(kLogger, "publisher_cache_evicted")
      .field("reason", "max_topics_exceeded")
      .field("topic", topic)
      .field("evicted_topic", evicted->key)
      .field("count", count)
      .field("max_topics", static_cast<int>(cache_limit_))
      .warn();
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "publish_request_failed");
    addPublishRequestFields(event.field("reason", "internal"), topic, requester_identity, &interface_type);
    event.field("error", exc.what()).error();
  }
}

void RosTopicPublisher::shutdown()
{
  // Flip the terminal bit before clearing cached handles so racing publish()
  // calls observe shutdown before they can repopulate bridge-owned cache state.
  if (is_shutdown_.exchange(true)) {
    return;
  }

  const std::size_t cached_count = cache_.size();
  LogEvent(kLogger, "topic_publisher_state_changed")
    .field("reason", "shutdown")
    .field("cached_publishers", cached_count)
    .info();

  cache_.clear();
}

}  // namespace livekit_ros2_bridge
