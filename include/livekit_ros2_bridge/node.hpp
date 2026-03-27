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

#include "livekit/room.h"
#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace livekit_ros2_bridge
{

class Node final : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Node() override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::unique_ptr<livekit::Room> room_;
};

}  // namespace livekit_ros2_bridge
