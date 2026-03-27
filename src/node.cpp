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

#include <stdexcept>
#include <utility>

#include "livekit_ros2_bridge/rpc_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace livekit_ros2_bridge
{

Node::Node(const rclcpp::NodeOptions & options)
: Node(options, makeLiveKitSession())
{}

Node::Node(const rclcpp::NodeOptions & options, std::unique_ptr<LiveKitSession> session)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  const Params params = param_listener_->get_params();
  rpc_controller_ = std::make_unique<RpcController>(get_logger(), params);
  RCLCPP_INFO(get_logger(), "Parameters loaded");

  session_ = std::move(session);
  if (session_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit session");
  }

  const std::string & url = params.livekit.url;
  const std::string & token = params.livekit.token;

  if (session_->connect(url, token)) {
    rpc_controller_->registerMethods(*session_);
  }
}

Node::~Node()
{
  if (session_ != nullptr) {
    if (rpc_controller_ != nullptr) {
      rpc_controller_->unregisterMethods(*session_);
    }
    session_->disconnect();
  }
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
