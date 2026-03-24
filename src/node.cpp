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

#include "livekit_ros2_bridge/node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace livekit_ros2_bridge {

Node::Node(const rclcpp::NodeOptions &options)
    : rclcpp::Node("livekit_ros2_bridge", options) {
  RCLCPP_INFO(get_logger(), "Node started");
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
