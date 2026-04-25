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
#include <cstdint>
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

namespace livekit_ros2_bridge
{

class RosTopicPublisher final
{
public:
  RosTopicPublisher(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    AccessPolicy policy);
  RosTopicPublisher(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    AccessPolicy policy,
    std::size_t max_topics);
  ~RosTopicPublisher();

  // Invalid or denied requests, ROS publisher errors, and calls after shutdown
  // are dropped without throwing. `request.message` is serialized ROS CDR bytes
  // for `request.interface_type`; first publish pins the topic to its graph type.
  void publish(const std::string & requester_identity, const RosPublishRequest & request);

  // Malformed payloads and missing requester identities are logged and dropped.
  void handlePayload(const std::string & requester_identity, const std::vector<std::uint8_t> & payload);

  void shutdown();

private:
  struct Entry
  {
    std::string type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
  };

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  rclcpp::Clock::SharedPtr clock_;
  AccessPolicy policy_;
  std::atomic<bool> is_shutdown_{false};
  std::size_t max_topics_;

  std::mutex mutex_;
  // shared_ptr lets an in-flight publish finish after shutdown() clears the map.
  std::unordered_map<std::string, Entry> publishers_;
};

}  // namespace livekit_ros2_bridge
