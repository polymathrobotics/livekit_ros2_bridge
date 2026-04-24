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
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "access_policy.hpp"
#include "protocol/topic_publish.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

namespace livekit
{
struct UserDataPacketEvent;
}  // namespace livekit

namespace livekit_ros2_bridge
{

// Best-effort LiveKit -> ROS topic publisher that caches concrete ROS
// publishers per topic. Failed publish attempts are logged and dropped rather
// than surfacing exceptions to the caller.
class RosTopicPublisher final
{
public:
  RosTopicPublisher(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    AccessPolicy access_policy);
  RosTopicPublisher(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    AccessPolicy access_policy,
    std::size_t max_topics);
  ~RosTopicPublisher();

  // Publishes best-effort: denied topics, type mismatches, shutdown, and ROS
  // publisher errors are logged and ignored without throwing to the caller.
  // `request.cdr` must already contain serialized ROS CDR bytes for
  // `request.interface_type`; publish() copies that payload directly into
  // rclcpp::SerializedMessage. Cold topics must already exist in the ROS graph
  // with exactly one interface type. After the first successful publish, later
  // requests are checked against the cached publisher/type instead of
  // consulting the graph again. Once the bounded publisher cache is full, new
  // topics are rejected until shutdown clears the cached handles.
  void publish(const std::string & requester_identity, const TopicPublishRequest & request);
  void handlePublishPacket(const std::string & requester_identity, const livekit::UserDataPacketEvent & event);

  // Idempotently rejects later publish() calls and clears the bridge-owned
  // cached publisher handles.
  void shutdown();

private:
  struct PublisherEntry
  {
    // Cache the validated interface type alongside the reusable publisher so
    // cache hits can reject mismatched requests without another graph lookup.
    std::string type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
  };

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  rclcpp::Clock::SharedPtr clock_;
  AccessPolicy access_policy_;
  // Terminal lifecycle bit shared with in-flight publish() calls. publish()
  // rechecks it before reusing or updating cache state so shutdown() does not
  // resurrect bridge-owned publishers after teardown begins.
  std::atomic<bool> is_shutdown_{false};
  std::size_t max_topics_;
  // Cache entries own the bridge's reusable publisher handles. The shared_ptr
  // lets an in-flight publish finish even if shutdown() clears the cache
  // concurrently.
  std::mutex publishers_mutex_;
  std::unordered_map<std::string, PublisherEntry> publishers_;
  // Test-only seam used to force deterministic failures or shutdown after
  // publisher resolution/creation but immediately before publish().
  std::function<void()> before_publish_handler_;
  // Test-only topic graph provider used to avoid mutating the real ROS graph in
  // narrow unit tests. publish() only consults it on cache misses.
  std::function<std::map<std::string, std::vector<std::string>>()> topic_graph_provider_;
};

}  // namespace livekit_ros2_bridge
