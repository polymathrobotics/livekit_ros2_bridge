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

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace livekit_ros2_bridge
{

class Runtime;

// Thin ROS component boundary that translates parameterized startup into one Runtime instance.
// Construction is eager: configuration loading and runtime startup both happen in the
// constructor, and failures are surfaced by throwing instead of leaving a partially started
// bridge behind.
class Node final : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // Defined out-of-line so shutdown can reset runtime_ while the rclcpp::Node base and its
  // interfaces are still alive.
  ~Node() override;

private:
  // Runtime keeps references into this node, so Node tears it down explicitly from the
  // destructor body while rclcpp state needed during shutdown is still alive.
  std::unique_ptr<Runtime> runtime_;
};

}  // namespace livekit_ros2_bridge
