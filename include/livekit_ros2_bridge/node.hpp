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

class RoomSession;
class Runtime;

// ROS component boundary that loads startup config and owns one bridge runtime.
class Node final : public rclcpp::Node
{
public:
  // Builds the default LiveKit session implementation and starts the runtime before returning.
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // Transfers ownership of an already-constructed session to the runtime. The session is started
  // during construction and stopped when this node is destroyed.
  Node(const rclcpp::NodeOptions & options, std::unique_ptr<RoomSession> session);
  ~Node() override;

private:
  // Keeps the bridge runtime alive for the full node lifetime, including shutdown ordering.
  std::unique_ptr<Runtime> runtime_;
};

}  // namespace livekit_ros2_bridge
