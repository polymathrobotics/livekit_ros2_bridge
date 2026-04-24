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

#include <exception>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "room_connection.hpp"
#include "runtime.hpp"
#include "runtime_config.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

class Node::Impl final
{
public:
  explicit Impl(Node & node)
  : runtime_(node, createRoomConnection(), loadRuntimeConfig(node.get_node_parameters_interface()))
  {}

private:
  Runtime runtime_;
};

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  try {
    pimpl_ = std::make_unique<Impl>(*this);
  } catch (...) {
    LogEvent(get_logger(), "node_startup_failed")
      .field("reason", "runtime_initialization_failed")
      .fieldException("error", std::current_exception())
      .error();
    throw;
  }
}

// Keep the destructor out-of-line so the opaque implementation is torn down while the
// rclcpp::Node base and its interfaces are still alive.
Node::~Node() = default;

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
