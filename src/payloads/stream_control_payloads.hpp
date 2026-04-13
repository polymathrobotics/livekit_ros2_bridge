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

#include "nlohmann/json_fwd.hpp"
#include "subscription_types.hpp"

namespace livekit_ros2_bridge
{

// Control-path subscription heartbeats carry lease-backed demands and report status. RPC
// request/response naming stays separate for RPC payloads such as service calls.

namespace stream_control_payloads
{

/// Parse one control heartbeat body from `ros.subscriptions.request`.
/// `session_id` is optional and trimmed; missing, null, or blank values are treated as absent.
/// `subscriptions` is required. `topic` names are normalized as ROS resource names, while
/// `configured_source` names are only trimmed because they are bridge-defined identifiers.
/// Duplicate canonical targets are coalesced in first-seen order, keeping the smallest non-zero
/// preferred interval. Wire integer intervals are clamped into `int`; later policy code may
/// further normalize values such as negatives.
SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body);

/// Serialize one active subscription status object for `ros.subscriptions.status`.
/// The wire shape always emits `status="active"`; partial health is reported via
/// `degraded_reason` instead of a separate status literal. The caller supplies the
/// already-resolved delivery mode and, for data deliveries, the effective applied interval.
/// Data payloads include `content_type="application/x-ros-cdr"` and `delivery.interval_ms`;
/// video payloads omit both.
nlohmann::json serializeSubscriptionStatus(const SubscriptionStatus & status);

}  // namespace stream_control_payloads

}  // namespace livekit_ros2_bridge
