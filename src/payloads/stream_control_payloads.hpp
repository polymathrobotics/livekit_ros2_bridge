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

/// Parse a heartbeat JSON object with optional `session_id` and required `subscriptions`.
/// Each heartbeat demand must contain required `kind` and `name` strings, plus an optional
/// `delivery_preferences.interval_ms` integer. `topic` names are normalized as ROS names,
/// `configured_source` names are trimmed only, duplicate canonical targets are coalesced, and
/// duplicate demand intervals keep the smallest non-zero value.
SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body);

/// Serialize one active subscription status object. Data deliveries include
/// `content_type="application/x-ros-cdr"` and `delivery.interval_ms`.
nlohmann::json serializeSubscriptionStatus(const SubscriptionStatus & subscription_status);

}  // namespace livekit_ros2_bridge
