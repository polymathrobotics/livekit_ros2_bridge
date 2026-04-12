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
#include <cstddef>
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

// Best-effort LiveKit -> ROS ingress writer that caches concrete ROS
// publishers per topic. Unlike DataTrackPublisher and VideoTrackPublisher, it
// does not own a LiveKit publication. Failures are logged and dropped rather
// than surfaced back through a retry or delivery guarantee.
class RosTopicWriter final
{
public:
  RosTopicWriter(rclcpp::Node & node, AccessPolicy access_policy);
  RosTopicWriter(rclcpp::Node & node, AccessPolicy access_policy, std::size_t max_cached_publishers);

  // Writes best-effort: denied topics, type mismatches, shutdown, and ROS
  // publisher errors are logged and ignored without throwing to the caller.
  void write(const std::string & requester_identity, const TopicPublishCommand & command);

  void shutdown();

private:
  static constexpr auto kEvictedPublisherWarningThrottlePeriod = std::chrono::seconds(5);

  // Test-only hook that runs after publisher resolution/creation and
  // immediately before the underlying ROS publisher publishes the message.
  void setBeforeWriteHookForTest(std::function<void()> hook);

  struct CachedPublisher
  {
    std::string interface_type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
  };

  std::string resolveTopicTypeOrThrow(const std::string & topic, const std::string & requested_interface_type) const;
  void writeWithResolvedPublisher(
    const std::string & topic, const std::string & interface_type, const rclcpp::SerializedMessage & serialized);

  rclcpp::Node & node_;
  AccessPolicy access_policy_;
  std::size_t max_cached_publishers_ = 0U;
  std::atomic<bool> is_shutdown_{false};
  BoundedLruCache<std::string, CachedPublisher> cached_publishers_;
  std::function<void()> before_write_hook_for_test_;
  EventThrottle evicted_publisher_warning_throttle_{kEvictedPublisherWarningThrottlePeriod};
};

}  // namespace livekit_ros2_bridge
