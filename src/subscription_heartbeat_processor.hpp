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
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>

#include "access_policy.hpp"
#include "nlohmann/json_fwd.hpp"
#include "payloads/stream_control_payloads.hpp"
#include "rclcpp/clock.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class RoomSession;
class SubscriptionRegistry;

// Resolves heartbeat lease ownership, including anonymous renewals that present a still-leased
// session_id, then renews the requested subscriptions and publishes one status envelope back.
class SubscriptionHeartbeatProcessor final
{
public:
  SubscriptionHeartbeatProcessor(
    SubscriptionRegistry & subscription_registry,
    RoomSession & session,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock);

  // Renews every requested stream for the resolved requester identity. Anonymous heartbeats are
  // accepted only when session_id already maps to an active requester lease.
  void process(const std::string & requester_identity, const SubscriptionHeartbeat & update);
  // Expires only the session_id leases used for anonymous-heartbeat fallback.
  void pruneExpiredSessionLeases();

private:
  struct SessionLease
  {
    std::string requester_identity;
    std::chrono::steady_clock::time_point expiry;
  };

  // Returns the requester identity that owns this heartbeat, binding or renewing session_id when
  // present. A missing requester_identity is accepted only for a known, unexpired session lease.
  std::optional<std::string> resolveRequesterIdentity(
    const std::string & requester_identity,
    const SubscriptionHeartbeat & update,
    std::chrono::steady_clock::time_point requester_lease_expiry);
  // Renews a session_id lease for exactly one requester identity until requester_lease_expiry.
  // Conflicts are rejected so a delayed or replayed heartbeat cannot steal another participant's
  // lease.
  bool renewRequesterSessionLease(
    const std::string & session_id,
    const std::string & requester_identity,
    std::chrono::steady_clock::time_point requester_lease_expiry);
  void publishSubscriptionStatus(
    const std::string & requester_identity,
    const std::optional<std::string> & session_id,
    std::chrono::steady_clock::time_point requester_lease_expiry,
    const nlohmann::json & streams);

  SubscriptionRegistry & subscription_registry_;
  RoomSession & room_session_;
  AccessPolicy access_policy_;
  rclcpp::Clock::SharedPtr clock_;
  // Tracks which requester identity is allowed to renew a session_id anonymously until expiry.
  std::unordered_map<std::string, SessionLease> session_leases_;
  EventThrottle session_conflict_throttle_{std::chrono::seconds(5)};
};

}  // namespace livekit_ros2_bridge
