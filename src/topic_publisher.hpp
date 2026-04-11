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

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "access_policy.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialized_message.hpp"
#include "topic_publish_command.hpp"
#include "utils/bounded_lru_cache.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

// Publishes serialized ROS messages for authorized LiveKit requesters while
// caching generic publishers per topic. Failures are logged and dropped rather
// than surfaced back through a retry or delivery guarantee.
class RosTopicPublisher final
{
public:
  RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, int max_topics);

  // Publishes best-effort: denied topics, type mismatches, shutdown, and
  // publisher errors are logged and ignored without throwing to the caller.
  void publish(const std::string & requester_identity, const TopicPublishCommand & command);

  void shutdown();

private:
  static constexpr auto kPublisherCacheEvictionThrottlePeriod = std::chrono::seconds(5);

  // Test-only hook that runs after publisher resolution/creation and
  // immediately before the underlying ROS publisher publishes the message.
  void setBeforePublishHookForTest(std::function<void()> hook);

  struct PublisherCacheEntry
  {
    std::string interface_type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher_handle;
  };

  std::string resolveTopicTypeOrThrow(const std::string & topic, const std::string & requested_interface_type) const;
  void publishWithPublisherCache(
    const std::string & topic, const std::string & interface_type, const rclcpp::SerializedMessage & serialized);

  rclcpp::Node & node_;
  AccessPolicy access_policy_;
  int max_topics_ = 0;
  std::atomic<bool> is_shutdown_{false};
  BoundedLruCache<std::string, PublisherCacheEntry> publishers_;
  std::function<void()> before_publish_hook_for_test_;
  EventThrottle publisher_cache_eviction_throttle_{kPublisherCacheEvictionThrottlePeriod};
};

}  // namespace livekit_ros2_bridge
