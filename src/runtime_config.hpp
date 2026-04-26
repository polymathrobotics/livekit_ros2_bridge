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
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "room_connection.hpp"
#include "subscription_qos.hpp"
#include "video/stream_spec.hpp"

namespace livekit_ros2_bridge
{

struct RuntimeConfig
{
  struct Watchdog
  {
    bool enabled = true;
    // Watchdog closes the room when SDK reconnect exceeds this window.
    std::chrono::milliseconds recovery_timeout{std::chrono::seconds(75)};
  };

  LiveKitConfig livekit;
  Watchdog watchdog;
  AccessPolicy access_policy;
  SubscriptionQosConfig subscription_qos;
  video::StreamConfig video_stream;
};

// Loads one ROS parameter snapshot; falls back to LIVEKIT_TOKEN when livekit.token is unset.
RuntimeConfig loadRuntimeConfig(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters);

}  // namespace livekit_ros2_bridge
