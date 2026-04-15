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
constexpr std::size_t kDefaultMaxTopics = 50U;
constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("topic_publisher");

}  // namespace

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy)
: RosTopicPublisher(node, std::move(access_policy), kDefaultMaxTopics)
{}

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, std::size_t max_topics)
: node_(node)
, access_policy_(std::move(access_policy))
, publishers_(max_topics)
{}

void RosTopicPublisher::publish(const std::string & requester_identity, const TopicPublishCommand & command)
{
  // TODO: Rename TopicPublishCommand/command to TopicPublishRequest/request. ROS uses
  // "message" for the thing published on a topic; this object is the publish request.
  const std::string & topic = command.topic;

  if (is_shutdown_.load()) {
    return;
  }

  if (!access_policy_.allows(AccessOperation::Publish, topic)) {
    LogEvent(kLogger, "publish_request_rejected")
      .field("reason", "forbidden")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .warnThrottle(*node_.get_clock(), kLogThrottle);

    return;
  }

  std::string type;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;
  try {
    // Cache hits deliberately skip the ROS graph. Once a publish succeeds, the
    // cached publisher pins the interface type for that topic until eviction or
    // shutdown clears the entry.
    if (const auto cached = publishers_.peek(topic); cached.has_value()) {
      type = cached->type;
      publisher = cached->publisher;
    } else {
      const auto topics = topic_graph_provider_ ? topic_graph_provider_() : node_.get_topic_names_and_types();
      type = requireSingleInterfaceType(topics, topic, "topic");
    }

    if (type != command.interface_type) {
      throw std::invalid_argument("type mismatch expected=" + type + " got=" + command.interface_type);
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "publish_request_rejected")
      .field("reason", "invalid_request")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", command.interface_type)
      .field("error", exc.what())
      .warnThrottle(*node_.get_clock(), kLogThrottle);

    return;
  }

  const bool had_cached_publisher = static_cast<bool>(publisher);

  // Control packets already carry a CDR payload, so copy the bytes directly
  // into SerializedMessage without a deserialize/serialize round trip.
  rclcpp::SerializedMessage serialized(command.cdr.size());
  auto & rcl_message = serialized.get_rcl_serialized_message();
  if (!command.cdr.empty()) {
    std::memcpy(rcl_message.buffer, command.cdr.data(), command.cdr.size());
  }
  rcl_message.buffer_length = command.cdr.size();

  try {
    if (!publisher) {
      const rclcpp::QoS qos(kPublisherDepth);
      publisher = node_.create_generic_publisher(topic, type, qos);
    }

    if (before_publish_handler_) {
      before_publish_handler_();
    }

    // before_publish_handler_ can trigger shutdown after publisher creation, so
    // recheck before sending bytes.
    if (is_shutdown_.load()) {
      return;
    }

    publisher->publish(serialized);
    // If shutdown is already visible here, bail out before touching cache state
    // that teardown is intentionally trying to drop.
    if (is_shutdown_.load()) {
      return;
    }

    // Refresh recency only after a successful publish.
    if (had_cached_publisher && publishers_.touch(topic)) {
      return;
    }

    // Enforce the cap only after the current publish succeeds.
    const auto evicted = publishers_.insertOrAssign(topic, PublisherEntry{type, std::move(publisher)});
    if (!evicted.has_value()) {
      return;
    }

    const std::size_t evicted_count = eviction_warning_throttle_.recordAndTakePendingCount();
    if (evicted_count == 0U) {
      return;
    }

    LogEvent(kLogger, "publisher_cache_evicted")
      .field("reason", "max_topics_exceeded")
      .field("topic", topic)
      .field("evicted_topic", evicted->key)
      .field("count", evicted_count)
      .field("max_topics", static_cast<int>(publishers_.capacity()))
      .warn();
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "publish_request_failed")
      .field("reason", "internal")
      .field("topic", topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", type)
      .field("error", exc.what())
      .error();
  }
}

void RosTopicPublisher::shutdown()
{
  // Flip the terminal bit before clearing cached handles so racing publish()
  // calls observe shutdown before they can repopulate bridge-owned cache state.
  if (is_shutdown_.exchange(true)) {
    return;
  }

  const std::size_t publisher_count = publishers_.size();
  LogEvent(kLogger, "topic_publisher_state_changed")
    .field("reason", "shutdown")
    .field("cached_publishers", publisher_count)
    .info();

  publishers_.clear();
}

}  // namespace livekit_ros2_bridge
