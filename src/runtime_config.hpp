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
#include "room_connection.hpp"
#include "subscription_qos.hpp"
#include "video_profiling.hpp"
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
    // Maximum time startup or any later reconnect may remain disconnected
    // before Runtime requests process shutdown.
    std::chrono::milliseconds fail_fast_disconnect_grace{std::chrono::minutes(10)};
  };

  Params params;
  LiveKitConfig livekit;
  HealthConfig health;
  AccessPolicy access_policy;
  SubscriptionQosConfig subscription_qos;
  VideoStreamConfig video_stream;
  VideoProfilingConfig video_profiling;
};

// Loads the bridge's startup-only configuration from parameters once. Later ROS parameter
// mutations are not observed.
// Throws std::invalid_argument for a null interface and std::runtime_error for invalid config.
RuntimeConfig loadRuntimeConfig(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters);

}  // namespace livekit_ros2_bridge
