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
#include <optional>

#include "nlohmann/json_fwd.hpp"
#include "subscription_types.hpp"

// Control-path subscription heartbeats carry lease-backed demands and report status. RPC
// request/response naming stays separate for RPC payloads such as service calls.
namespace livekit_ros2_bridge::wire::subscriptions
{

/// Parse one control heartbeat body from `ros.subscriptions.request`.
/// `session_id` is optional and trimmed; missing, null, or blank values are treated as absent.
/// `subscriptions` is required. `topic` names are normalized as ROS resource names, while
/// `configured_source` names are only trimmed because they are bridge-defined identifiers.
/// Duplicate canonical targets are coalesced in first-seen order, keeping the smallest non-zero
/// preferred interval. Wire integer intervals are clamped into `int`; later policy code may
/// further normalize values such as negatives.
SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body);

struct SubscriptionStatusLease
{
  std::string session_id;
  std::chrono::steady_clock::time_point expiry;
};

/// Serialize the full `ros.subscriptions.status` response body from reported status DTOs and
/// optional lease metadata. The serializer owns the per-entry `active`/`error` mapping and the
/// top-level envelope shape, including `lease_expires_in_ms`.
nlohmann::json serializeSubscriptionStatuses(
  const std::vector<SubscriptionReportedStatus> & statuses,
  const std::optional<SubscriptionStatusLease> & lease,
  std::chrono::steady_clock::time_point now);

/// Serialize the full `ros.subscriptions.status` response body using `steady_clock::now()` for
/// `lease_expires_in_ms`.
nlohmann::json serializeSubscriptionStatuses(
  const std::vector<SubscriptionReportedStatus> & statuses, const std::optional<SubscriptionStatusLease> & lease);

}  // namespace livekit_ros2_bridge::wire::subscriptions
