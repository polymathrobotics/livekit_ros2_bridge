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

#include "topic_publisher.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "utils/interface_types.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kPublisherDepth = 10U;
const auto kTopicPublisherLogger = rclcpp::get_logger("topic_publisher");

rclcpp::SerializedMessage toSerializedMessage(const std::vector<std::uint8_t> & payload)
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

RosTopicPublisher::RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, int max_topics)
: node_(node)
, access_policy_(std::move(access_policy))
, max_topics_(max_topics)
{
  if (max_topics < 0) {
    throw std::invalid_argument("max_topics must be >= 0");
  }
}

void RosTopicPublisher::publish(const std::string & requester_identity, const TopicPublishCommand & command)
{
  const std::string & topic = command.topic;

  // Publish commands come from a streaming control path, so this component is
  // intentionally best-effort: invalid or late commands are dropped after logging.
  if (is_shutdown_.load()) {
    RCLCPP_WARN(
      kTopicPublisherLogger,
      "event=publish_request_rejected reason=shutdown resource=topics topic=%s requester_identity=%s "
      "interface_type=%s",
      topic.c_str(),
      requester_identity.c_str(),
      command.interface_type.c_str());
    return;
  }

  if (!access_policy_.allows(AccessOperation::Publish, topic)) {
    RCLCPP_WARN(
      kTopicPublisherLogger,
      "event=publish_request_rejected reason=forbidden resource=topics topic=%s requester_identity=%s "
      "interface_type=%s",
      topic.c_str(),
      requester_identity.c_str(),
      command.interface_type.c_str());
    return;
  }

  std::string interface_type;
  try {
    interface_type = resolveTopicTypeOrThrow(topic, command.interface_type);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      kTopicPublisherLogger,
      "event=publish_request_rejected reason=invalid_request resource=topics topic=%s "
      "requester_identity=%s interface_type=%s error=%s",
      topic.c_str(),
      requester_identity.c_str(),
      command.interface_type.c_str(),
      exc.what());
    return;
  }

  rclcpp::SerializedMessage serialized = toSerializedMessage(command.cdr_payload);

  try {
    publishWithPublisherCache(topic, interface_type, serialized);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      kTopicPublisherLogger,
      "event=publish_request_failed reason=internal resource=topics topic=%s requester_identity=%s "
      "interface_type=%s error=%s",
      topic.c_str(),
      requester_identity.c_str(),
      interface_type.c_str(),
      exc.what());
    return;
  }
}

void RosTopicPublisher::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }

  const std::size_t cached_publishers = publishers_.size();
  RCLCPP_INFO(
    kTopicPublisherLogger,
    "event=topic_publisher_state_changed reason=shutdown action=clear_cached_publishers cached_publishers=%zu",
    cached_publishers);

  auto publishers = std::move(publishers_);
  publishers_.clear();
  lru_topics_.clear();

  for (auto & entry : publishers) {
    entry.second.publisher_handle.reset();
  }
}

std::string RosTopicPublisher::resolveTopicTypeOrThrow(
  const std::string & topic, const std::string & requested_interface_type) const
{
  std::string expected_type;

  const auto publisher_it = publishers_.find(topic);
  if (publisher_it != publishers_.end()) {
    // Reuse the cache's interface type once a publisher exists so later
    // commands stay consistent even if graph introspection lags that creation.
    expected_type = publisher_it->second.interface_type;
  } else {
    expected_type = requireUniqueInterfaceType(node_.get_topic_names_and_types(), topic, "topic");
  }

  if (expected_type != requested_interface_type) {
    throw std::invalid_argument("type mismatch expected=" + expected_type + " got=" + requested_interface_type);
  }
  return expected_type;
}

void RosTopicPublisher::publishWithPublisherCache(
  const std::string & topic, const std::string & interface_type, const rclcpp::SerializedMessage & serialized)
{
  auto publisher_it = publishers_.find(topic);
  const bool was_cached = publisher_it != publishers_.end();
  if (!was_cached) {
    const rclcpp::QoS qos(kPublisherDepth);
    auto publisher = node_.create_generic_publisher(topic, interface_type, qos);
    lru_topics_.push_back(topic);
    try {
      publisher_it =
        publishers_
          .emplace(topic, PublisherCacheEntry{interface_type, std::move(publisher), std::prev(lru_topics_.end())})
          .first;
    } catch (...) {
      lru_topics_.pop_back();
      throw;
    }
  }

  try {
    if (before_publish_hook_for_test_) {
      before_publish_hook_for_test_();
    }
    publisher_it->second.publisher_handle->publish(serialized);
  } catch (...) {
    if (!was_cached) {
      eraseCachedPublisher(topic);
    }
    throw;
  }

  // Refresh recency only after a successful publish so failed attempts do not
  // change bounded-cache residency.
  if (was_cached) {
    lru_topics_.splice(lru_topics_.end(), lru_topics_, publisher_it->second.lru_position);
  }

  // Enforce the cap after serving the current command: the publish succeeds and
  // an older cached publisher is discarded to make room for future use.
  while (max_topics_ != 0 && publishers_.size() > static_cast<std::size_t>(max_topics_)) {
    const std::string evicted_topic = lru_topics_.front();
    eraseCachedPublisher(evicted_topic);
    RCLCPP_WARN(
      kTopicPublisherLogger,
      "Publisher topic cap reached; evicted topic=%s to allow topic=%s",
      evicted_topic.c_str(),
      topic.c_str());
  }
}

void RosTopicPublisher::setBeforePublishHookForTest(std::function<void()> hook)
{
  before_publish_hook_for_test_ = std::move(hook);
}

void RosTopicPublisher::eraseCachedPublisher(const std::string & topic)
{
  const auto publisher_it = publishers_.find(topic);
  if (publisher_it == publishers_.end()) {
    return;
  }

  lru_topics_.erase(publisher_it->second.lru_position);
  publisher_it->second.publisher_handle.reset();
  publishers_.erase(publisher_it);
}

}  // namespace livekit_ros2_bridge
