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

namespace livekit_ros2_bridge::wire::cdr
{

/// Parse the shared CDR envelope stored at `body[field]`.
/// `field` names the protocol-specific outer key (`message`, `request`, `response`) and
/// is used for lookup plus diagnostics only; the nested envelope contract is otherwise the same
/// everywhere.
/// This helper validates `content_type="application/x-ros-cdr"` and padded standard-base64
/// `payload_base64`, but it intentionally allows the decoded byte vector to be empty so higher-level
/// payload parsers can decide whether a given endpoint permits empty serialized messages.
/// Throws `std::invalid_argument` when the field is missing, mistyped, uses a different
/// content type, or contains invalid base64.
std::vector<std::uint8_t> parse(const nlohmann::json & body, const char * field);

/// Serialize raw ROS 2 CDR bytes into the canonical envelope shared across protocol surfaces.
/// Empty input serializes as an empty `payload_base64` string so `serialize` and `parse` remain
/// lossless even when the caller handles "must not be empty" as a separate policy decision.
nlohmann::json serialize(const std::vector<std::uint8_t> & bytes);

}  // namespace livekit_ros2_bridge::wire::cdr
