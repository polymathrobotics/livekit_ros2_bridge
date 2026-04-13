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

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  const auto logger = get_logger();
  LogEvent(logger, "node_startup_begin").field("phase", "startup").info();

  RuntimeConfig config;
  std::string room;
  try {
    config = loadRuntimeConfig(get_node_parameters_interface());
    // Copy the room before Runtime takes ownership of config so later startup failures can
    // still attribute the error to the intended room.
    room = config.room_connection_config.room;
  } catch (const std::exception & exc) {
    LogEvent(logger, "node_startup_failed")
      .field("phase", "startup")
      .field("reason", "runtime_config_load_failed")
      .field("error", exc.what())
      .error();
    throw;
  } catch (...) {
    LogEvent(logger, "node_startup_failed")
      .field("phase", "startup")
      .field("reason", "runtime_config_load_failed")
      .field("error", "unknown_exception")
      .error();
    throw;
  }

  // Keep configuration loading separate from runtime startup so startup logs distinguish
  // invalid parameters from room-connection or runtime initialization failures.
  try {
    runtime_ = std::make_unique<Runtime>(*this, createRoomConnection(), std::move(config));
  } catch (const std::exception & exc) {
    LogEvent(logger, "node_startup_failed")
      .field("phase", "startup")
      .field("reason", "runtime_initialization_failed")
      .fieldOr("room", room, "<unset>")
      .field("error", exc.what())
      .error();
    throw;
  } catch (...) {
    LogEvent(logger, "node_startup_failed")
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
  // Tear Runtime down before the component reports shutdown complete so its teardown can still use
  // this node's logger and interfaces.
  runtime_.reset();
  LogEvent(get_logger(), "node_shutdown_complete").field("phase", "shutdown").info();
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
