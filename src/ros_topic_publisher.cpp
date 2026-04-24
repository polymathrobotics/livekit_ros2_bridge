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
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "livekit/room_event_types.h"
#include "protocol/topic_publish_json.hpp"
#include "rclcpp/create_generic_publisher.hpp"
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

RosTopicPublisher::RosTopicPublisher(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  AccessPolicy access_policy)
: RosTopicPublisher(std::move(topics), std::move(graph), std::move(clock), std::move(access_policy), kDefaultMaxTopics)
{}

RosTopicPublisher::RosTopicPublisher(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  AccessPolicy access_policy,
  std::size_t max_topics)
: topics_(std::move(topics))
, graph_(std::move(graph))
, clock_(std::move(clock))
, access_policy_(std::move(access_policy))
, max_topics_(max_topics)
{}

RosTopicPublisher::~RosTopicPublisher()
{
  shutdown();
}

void RosTopicPublisher::handlePublishPacket(
  const std::string & requester_identity, const livekit::UserDataPacketEvent & event)
{
  if (requester_identity.empty()) {
    LogEvent(kLogger, "packet_rejected")
      .field("reason", "missing_requester_identity")
      .fieldOr("requester_identity", requester_identity)
      .warnThrottle(*clock_, kLogThrottle);
    return;
  }

  std::optional<TopicPublishRequest> request;
  try {
    request = protocol::topic_publish::parse(event.data);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "packet_rejected")
      .field("reason", "invalid_publish_request")
      .fieldOr("requester_identity", requester_identity)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
    return;
  }

  publish(requester_identity, *request);
}

void RosTopicPublisher::publish(const std::string & requester_identity, const TopicPublishRequest & request)
{
  if (is_shutdown_.load()) {
    return;
  }

  std::string ros_topic;
  try {
    ros_topic = topics_->resolve_topic_name(request.ros_topic);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "publish_request_rejected")
      .field("reason", "invalid_request")
      .field("topic", request.ros_topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", request.interface_type)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  if (!access_policy_.allows(AccessOperation::Publish, ros_topic)) {
    LogEvent(kLogger, "publish_request_rejected")
      .field("reason", "forbidden")
      .field("topic", ros_topic)
      .field("requester_identity", requester_identity)
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  std::string type;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;

  try {
    // Cache hits deliberately skip the ROS graph. Once a publish succeeds, a
    // cached publisher pins the interface type for that topic until shutdown
    // clears the entry.
    {
      std::lock_guard<std::mutex> lock(publishers_mutex_);
      const auto cached = publishers_.find(ros_topic);
      if (cached != publishers_.end()) {
        type = cached->second.type;
        publisher = cached->second.publisher;
      } else if (publishers_.size() >= max_topics_) {
        LogEvent(kLogger, "publish_request_rejected")
          .field("reason", "publisher_cache_full")
          .field("topic", ros_topic)
          .field("requester_identity", requester_identity)
          .field("max_topics", max_topics_)
          .warnThrottle(*clock_, kLogThrottle);
        return;
      }
    }

    if (!publisher) {
      const auto topics = topic_graph_provider_ ? topic_graph_provider_() : graph_->get_topic_names_and_types();
      type = requireSingleInterfaceType(topics, ros_topic, "topic");
    }

    if (type != request.interface_type) {
      throw std::invalid_argument("type mismatch expected=" + type + " got=" + request.interface_type);
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "publish_request_rejected")
      .field("reason", "invalid_request")
      .field("topic", ros_topic)
      .field("requester_identity", requester_identity)
      .field("interface_type", request.interface_type)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  const bool had_cached_publisher = static_cast<bool>(publisher);

  try {
    if (!publisher) {
      const rclcpp::QoS qos(kPublisherDepth);
      publisher = rclcpp::create_generic_publisher(topics_, ros_topic, type, qos);
    }

    if (before_publish_handler_) {
      before_publish_handler_();
    }

    // before_publish_handler_ can trigger shutdown after publisher creation, so
    // recheck before sending bytes.
    if (is_shutdown_.load()) {
      return;
    }

    publisher->publish(request.message);
    // If shutdown is already visible here, bail out before touching cache state
    // that teardown is intentionally trying to drop.
    if (is_shutdown_.load()) {
      return;
    }

    if (!had_cached_publisher) {
      // Cache only after a successful publish. Recheck shutdown and size under
      // the lock so a racing publish cannot push the map over max_topics_.
      std::lock_guard<std::mutex> lock(publishers_mutex_);
      if (is_shutdown_.load()) {
        return;
      }

      if (publishers_.size() < max_topics_) {
        publishers_.emplace(ros_topic, PublisherEntry{type, std::move(publisher)});
      }
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "publish_request_failed")
      .field("reason", "internal")
      .field("topic", ros_topic)
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

  std::size_t publisher_count = 0U;
  {
    std::lock_guard<std::mutex> lock(publishers_mutex_);
    publisher_count = publishers_.size();
    publishers_.clear();
  }

  LogEvent(kLogger, "topic_publisher_state_changed")
    .field("reason", "shutdown")
    .field("cached_publishers", publisher_count)
    .info();
}

}  // namespace livekit_ros2_bridge
