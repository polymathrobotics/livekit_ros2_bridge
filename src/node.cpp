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

namespace
{

const char * activeExceptionMessage() noexcept
{
  try {
    throw;
  } catch (const std::exception & exc) {
    return exc.what();
  } catch (...) {
    return "unknown_exception";
  }
}

}  // namespace

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  const auto logger = get_logger();

  RuntimeConfig config;
  std::string room;
  try {
    config = loadRuntimeConfig(get_node_parameters_interface());
    // Copy the room before Runtime takes ownership of config so later startup failures can
    // still attribute the error to the intended room.
    room = config.room_connection_config.room;
  } catch (...) {
    LogEvent(logger, "node_startup_failed")
      .field("reason", "runtime_config_load_failed")
      .field("error", activeExceptionMessage())
      .error();
    throw;
  }

  // Keep configuration loading separate from runtime startup so startup logs distinguish
  // invalid parameters from room-connection or runtime initialization failures.
  try {
    runtime_ = std::make_unique<Runtime>(*this, createRoomConnection(), std::move(config));
  } catch (...) {
    LogEvent(logger, "node_startup_failed")
      .field("reason", "runtime_initialization_failed")
      .fieldOr("room", room, "<unset>")
      .field("error", activeExceptionMessage())
      .error();
    throw;
  }
}

Node::~Node()
{
  // Tear Runtime down before the component reports shutdown complete so its teardown can still use
  // this node's logger and interfaces.
  runtime_.reset();
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
