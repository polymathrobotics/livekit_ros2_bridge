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
#include <vector>

#include "nlohmann/json_fwd.hpp"

namespace livekit_ros2_bridge
{

/// Parse `body[field_name]` as a CDR payload object with
/// `content_type="application/x-ros-cdr"` and padded standard-base64 `payload_base64`.
/// Throws `std::invalid_argument` when the field is missing, mistyped, uses a different
/// content type, or contains invalid base64.
std::vector<std::uint8_t> parseCdrPayload(const nlohmann::json & body, const char * field_name);

/// Serialize raw ROS 2 CDR bytes as `{ "content_type", "payload_base64" }`.
nlohmann::json serializeCdrPayload(const std::vector<std::uint8_t> & payload);

}  // namespace livekit_ros2_bridge
