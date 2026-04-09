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

namespace livekit_ros2_bridge
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("livekit_ros2_bridge", options)
{
  RCLCPP_INFO(get_logger(), "event=node_startup_begin phase=startup");
  RuntimeConfig runtime_config = [this]() {
    try {
      return loadRuntimeConfig(get_node_parameters_interface(), get_name());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(
        get_logger(), "event=node_startup_failed phase=startup reason=runtime_config_load_failed error=%s", exc.what());
      throw;
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(),
        "event=node_startup_failed phase=startup reason=runtime_config_load_failed error=unknown_exception");
      throw;
    }
  }();

  const char * room =
    runtime_config.connect_config.room.empty() ? "<unset>" : runtime_config.connect_config.room.c_str();
  const char * identity =
    runtime_config.connect_config.identity.empty() ? "<unset>" : runtime_config.connect_config.identity.c_str();
  const char * sidecar_enabled = runtime_config.video_sidecar_config.has_value() ? "true" : "false";

  try {
    runtime_ = std::make_unique<Runtime>(*this, makeRoomSession(), std::move(runtime_config));
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      get_logger(),
      "event=node_startup_failed phase=startup reason=runtime_initialization_failed room=%s identity=%s "
      "sidecar_enabled=%s error=%s",
      room,
      identity,
      sidecar_enabled,
      exc.what());
    throw;
  } catch (...) {
    RCLCPP_ERROR(
      get_logger(),
      "event=node_startup_failed phase=startup reason=runtime_initialization_failed room=%s identity=%s "
      "sidecar_enabled=%s error=unknown_exception",
      room,
      identity,
      sidecar_enabled);
    throw;
  }

  RCLCPP_INFO(
    get_logger(),
    "event=node_ready phase=startup room=%s identity=%s sidecar_enabled=%s",
    room,
    identity,
    sidecar_enabled);
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "event=node_shutdown_start phase=shutdown");
  runtime_.reset();
  RCLCPP_INFO(get_logger(), "event=node_shutdown_complete phase=shutdown");
}

}  // namespace livekit_ros2_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(livekit_ros2_bridge::Node)
