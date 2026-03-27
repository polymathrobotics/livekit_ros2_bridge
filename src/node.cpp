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

#include "livekit/livekit.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace livekit_ros2_bridge
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  livekit::initialize();
  RCLCPP_INFO(get_logger(), "LiveKit SDK initialized");

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
  RCLCPP_INFO(get_logger(), "Parameters loaded");

  const std::string & url = params_.livekit.url;
  const std::string & token = params_.livekit.token;

  room_ = std::make_unique<livekit::Room>();
  livekit::RoomOptions room_options;
  room_options.auto_subscribe = true;

  RCLCPP_INFO(get_logger(), "Connecting to LiveKit room at %s", url.c_str());
  const bool connected = room_->Connect(url, token, room_options);
  if (connected) {
    RCLCPP_INFO(get_logger(), "Connected to LiveKit room");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to connect to LiveKit room");
  }
}

Node::~Node()
{
  room_.reset();
  livekit::shutdown();
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
