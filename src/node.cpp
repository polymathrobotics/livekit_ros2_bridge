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
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "room_session.hpp"
#include "runtime.hpp"
#include "runtime_config.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  LogEvent(get_logger(), "node_startup_begin").field("phase", "startup").info();
  RuntimeConfig runtime_config = [this]() {
    try {
      return loadRuntimeConfig(get_node_parameters_interface());
    } catch (const std::exception & exc) {
      LogEvent(get_logger(), "node_startup_failed")
        .field("phase", "startup")
        .field("reason", "runtime_config_load_failed")
        .field("error", exc.what())
        .error();
      throw;
    } catch (...) {
      LogEvent(get_logger(), "node_startup_failed")
        .field("phase", "startup")
        .field("reason", "runtime_config_load_failed")
        .field("error", "unknown_exception")
        .error();
      throw;
    }
  }();

  const std::string room = runtime_config.room_connection_config.room;
  try {
    runtime_ = std::make_unique<Runtime>(*this, makeRoomSession(), std::move(runtime_config));
  } catch (const std::exception & exc) {
    LogEvent(get_logger(), "node_startup_failed")
      .field("phase", "startup")
      .field("reason", "runtime_initialization_failed")
      .fieldOr("room", room, "<unset>")
      .field("error", exc.what())
      .error();
    throw;
  } catch (...) {
    LogEvent(get_logger(), "node_startup_failed")
      .field("phase", "startup")
      .field("reason", "runtime_initialization_failed")
      .fieldOr("room", room, "<unset>")
      .field("error", "unknown_exception")
      .error();
    throw;
  }
}

Node::~Node()
{
  LogEvent(get_logger(), "node_shutdown_start").field("phase", "shutdown").info();
  runtime_.reset();
  LogEvent(get_logger(), "node_shutdown_complete").field("phase", "shutdown").info();
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
