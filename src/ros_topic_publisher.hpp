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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "access_policy.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/node.hpp"
#include "topic_publish_command.hpp"
#include "utils/event_throttle.hpp"
#include "utils/lru_cache.hpp"

namespace livekit_ros2_bridge
{

// Best-effort LiveKit -> ROS topic publisher that caches concrete ROS
// publishers per topic. Failed publish attempts are logged and dropped rather
// than surfacing exceptions to the caller.
class RosTopicPublisher final
{
public:
  RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy);
  RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, std::size_t max_topics);

  // Publishes best-effort: denied topics, type mismatches, shutdown, and ROS
  // publisher errors are logged and ignored without throwing to the caller.
  // `command.cdr` must already contain serialized ROS CDR bytes for
  // `command.interface_type`; publish() copies that payload directly into
  // rclcpp::SerializedMessage. Cold topics must already exist in the ROS graph
  // with exactly one interface type. After the first successful publish, later
  // requests are checked against the cached publisher/type instead of
  // consulting the graph again, and shutdown() keeps those cached handles from
  // being recreated once teardown starts.
  void publish(const std::string & requester_identity, const TopicPublishCommand & command);

  // Idempotently rejects later publish() calls and clears the bridge-owned
  // cached publisher handles.
  void shutdown();

private:
  static constexpr auto kEvictedPublisherWarningThrottlePeriod = std::chrono::seconds(5);

  struct PublisherEntry
  {
    // Cache the validated interface type alongside the reusable publisher so
    // cache hits can reject mismatched commands without another graph lookup.
    std::string type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
  };

  rclcpp::Node & node_;
  AccessPolicy access_policy_;
  std::size_t max_topics_ = 0U;
  // Terminal lifecycle bit shared with in-flight publish() calls. publish()
  // rechecks it before reusing or updating cache state so shutdown() does not
  // resurrect bridge-owned publishers after teardown begins.
  std::atomic<bool> is_shutdown_{false};
  // Cache entries own the bridge's reusable publisher handles. The shared_ptr
  // lets an in-flight publish finish even if its topic is evicted or
  // shutdown() clears the cache concurrently.
  LruCache<std::string, PublisherEntry> publishers_;
  // Test-only seam used to force deterministic failures or shutdown after
  // publisher resolution/creation but immediately before publish().
  std::function<void()> before_publish_handler_;
  // Test-only topic graph provider used to avoid mutating the real ROS graph in
  // narrow unit tests. publish() only consults it on cache misses.
  std::function<std::map<std::string, std::vector<std::string>>()> topic_graph_provider_;
  // Coalesces bursts of LRU evictions into periodic warnings while preserving
  // how many evictions were suppressed between log lines.
  EventThrottle eviction_warning_throttle_{kEvictedPublisherWarningThrottlePeriod};
};

}  // namespace livekit_ros2_bridge
