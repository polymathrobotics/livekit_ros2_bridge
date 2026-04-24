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

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

// Typed service-call request shared by the RPC boundary and ROS runtime.
struct ServiceCallRequest
{
  // Normalized absolute ROS service name.
  std::string service;
  // Optional `pkg/srv/Type` hint. Empty values let the caller resolve the type from the ROS graph.
  std::string interface_type;
  // Serialized request bytes. Empty payloads are invalid.
  std::vector<std::uint8_t> payload;
  // Caller-supplied timeout in milliseconds when present. The runtime maps omitted or non-positive
  // values to its default deadline.
  std::optional<int> timeout_ms;
};

struct ServiceCallResponse
{
  std::string service;
  std::string interface_type;
  // The payload stays serialized so runtime callers can forward arbitrary service types without
  // templating on generated ROS interfaces.
  std::vector<std::uint8_t> payload;
};

}  // namespace livekit_ros2_bridge
