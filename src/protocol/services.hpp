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
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/serialized_message.hpp"

namespace livekit_ros2_bridge
{

struct ServiceCallRequest
{
  std::string name;
  // Optional `pkg/srv/Type` hint. Empty means resolve the type from the ROS graph.
  std::string interface_type;
  // Empty payloads are invalid.
  rclcpp::SerializedMessage payload;
  // Omitted or non-positive values use the runtime default.
  std::optional<std::chrono::milliseconds> timeout;
};

struct ServiceCallResponse
{
  std::string name;
  std::string interface_type;
  // Serialized so callers can forward arbitrary service types without generated ROS interfaces.
  std::vector<std::uint8_t> payload;
};

}  // namespace livekit_ros2_bridge
