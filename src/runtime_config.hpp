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

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "access_policy.hpp"
#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "room_session.hpp"
#include "subscription_qos.hpp"
#include "video_stream_config.hpp"

namespace livekit_ros2_bridge
{

// Immutable startup configuration derived once from ROS parameters and then shared across the
// runtime and reconnect loop.
struct RuntimeConfig
{
  struct HealthConfig
  {
    bool fail_fast_enabled = true;
    std::chrono::milliseconds fail_fast_disconnect_grace{std::chrono::minutes(10)};
  };

  Params loaded_params;
  RoomConnectionConfig room_connection_config;
  std::string access_token;
  HealthConfig health_config;
  AccessPolicy access_policy;
  SubscriptionQosConfig subscription_qos_config;
  VideoStreamConfig video_stream_config;
};

// Loads and validates the bridge's startup-only configuration from parameters, deriving the
// connection settings, startup access token, access policy, and declared video stream config in
// one pass before Runtime starts.
RuntimeConfig loadRuntimeConfig(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters_interface);

}  // namespace livekit_ros2_bridge
