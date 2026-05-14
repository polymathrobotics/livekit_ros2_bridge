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
#include <cstddef>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "protocol/topic_publish_json.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/create_generic_publisher.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "ros_interfaces/graph_lookup.hpp"
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
  AccessPolicy policy)
: RosTopicPublisher(std::move(topics), std::move(graph), std::move(clock), std::move(policy), kDefaultMaxTopics)
{}

RosTopicPublisher::RosTopicPublisher(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  AccessPolicy policy,
  std::size_t max_topics)
: topics_(std::move(topics))
, graph_(std::move(graph))
, clock_(std::move(clock))
, policy_(std::move(policy))
, max_topics_(max_topics)
{}

RosTopicPublisher::~RosTopicPublisher()
{
  shutdown();
}

void RosTopicPublisher::handlePayload(const std::string & requester_identity, const std::vector<std::uint8_t> & payload)
{
  if (requester_identity.empty()) {
    LogEvent(kLogger, "livekit_packet_rejected")
      .field("reason", "missing_requester_identity")
      .warnThrottle(*clock_, kLogThrottle);
    return;
  }

  try {
    publish(requester_identity, protocol::topic_publish::parse(payload));
  } catch (const std::exception & exc) {
    const auto * validation = dynamic_cast<const protocol::ValidationError *>(&exc);
    LogEvent(kLogger, "livekit_packet_rejected")
      .field("reason", "invalid_publish_request")
      .fieldOr("requester_identity", requester_identity)
      .fieldOr("request_field", validation == nullptr ? std::string_view{} : validation->field())
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
    return;
  }
}

void RosTopicPublisher::publish(const std::string & requester_identity, const RosPublishRequest & request)
{
  if (is_shutdown_.load()) {
    return;
  }

  std::string topic;
  try {
    topic = topics_->resolve_topic_name(request.ros_topic);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "invalid_request")
      .fieldOr("topic", request.ros_topic)
      .fieldOr("requester_identity", requester_identity)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  if (!policy_.allows(AccessOperation::Publish, topic)) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "forbidden")
      .fieldOr("topic", topic)
      .fieldOr("requester_identity", requester_identity)
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  std::string type;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;

  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto entry = publishers_.find(topic);
      if (entry != publishers_.end()) {
        type = entry->second.type;
        publisher = entry->second.publisher;
      } else if (publishers_.size() >= max_topics_) {
        LogEvent(kLogger, "topic_publish_request_rejected")
          .field("reason", "publisher_cache_full")
          .fieldOr("topic", topic)
          .fieldOr("requester_identity", requester_identity)
          .field("max_topics", max_topics_)
          .warnThrottle(*clock_, kLogThrottle);
        return;
      }
    }

    if (!publisher) {
      type = ros_interfaces::requireSingleType(graph_->get_topic_names_and_types(), topic, "topic");
    }

    if (type != request.interface_type) {
      throw std::invalid_argument("type mismatch expected=" + type + " got=" + request.interface_type);
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "invalid_request")
      .fieldOr("topic", topic)
      .fieldOr("requester_identity", requester_identity)
      .fieldOr("interface_type", request.interface_type)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);

    return;
  }

  const bool was_cached = static_cast<bool>(publisher);

  try {
    if (!publisher) {
      const rclcpp::QoS qos(kPublisherDepth);
      publisher = rclcpp::create_generic_publisher(topics_, topic, type, qos);
    }

    // Honor concurrent shutdown before publishing and before first cache insertion.
    if (is_shutdown_.load()) {
      return;
    }

    publisher->publish(request.message);
    if (is_shutdown_.load()) {
      return;
    }

    if (!was_cached) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_.load()) {
        return;
      }

      if (publishers_.size() < max_topics_) {
        publishers_.emplace(topic, Entry{type, std::move(publisher)});
      }
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "topic_publish_request_failed")
      .fieldOr("topic", topic)
      .fieldOr("requester_identity", requester_identity)
      .fieldOr("interface_type", type)
      .field("error", exc.what())
      .error();
  }
}

void RosTopicPublisher::shutdown()
{
  // Set the terminal bit before clearing cached handles so in-flight publish()
  // calls cannot repopulate bridge-owned cache state.
  if (is_shutdown_.exchange(true)) {
    return;
  }

  std::size_t count = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count = publishers_.size();
    publishers_.clear();
  }

  if (count == 0U) {
    return;
  }

  LogEvent(kLogger, "topic_publisher_state_changed")
    .field("reason", "shutdown")
    .field("cached_publishers", count)
    .info();
}

}  // namespace livekit_ros2_bridge
