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

#include "protocol/subscriptions.hpp"

// Subscription control-plane messages carry lease-backed demands and report status. RPC
// request/response naming stays separate for RPC payloads such as service calls.
namespace livekit_ros2_bridge::protocol::subscriptions
{

/// Parse one subscription control-plane heartbeat payload from `lkros.heartbeat`.
/// `session_id` is optional and trimmed; missing, null, or blank values are treated as absent.
/// `subscriptions` is required. `topic` names are trimmed, expanded, and validated by rclcpp,
/// while `other_video` names are only trimmed because they are bridge-defined identifiers.
/// Duplicate canonical targets are coalesced in first-seen order, keeping the smallest non-zero
/// preferred interval. Protocol integer intervals are clamped into `int`; later policy code may
/// further normalize values such as negatives.
SubscriptionHeartbeat parseHeartbeat(const std::vector<std::uint8_t> & payload);

/// Serialize the full `lkros.status` payload from a typed status-report DTO.
/// `lease_expires_in_ms` is serialized whenever `report.lease_expiry` is present, even if
/// `report.session_id` is absent. The serializer owns the per-entry `active`/`error` mapping and
/// the top-level envelope shape.
std::string serializeStatusReport(const SubscriptionStatusReport & report, std::chrono::steady_clock::time_point now);

/// Serialize the full `lkros.status` payload using `steady_clock::now()` for
/// `lease_expires_in_ms`.
std::string serializeStatusReport(const SubscriptionStatusReport & report);

}  // namespace livekit_ros2_bridge::protocol::subscriptions
