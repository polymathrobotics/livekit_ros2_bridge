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

class RoomConnection;
class SubscriptionRegistry;

// Resolves heartbeat lease ownership, including anonymous renewals that present a still-leased
// wire `session_id`, then renews the requested subscriptions and publishes one status envelope
// back.
class SubscriptionHeartbeatProcessor final
{
public:
  SubscriptionHeartbeatProcessor(
    SubscriptionRegistry & subscription_registry,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock);

  // Renews every requested stream for the resolved requester identity. Anonymous heartbeats are
  // accepted only when the wire session_id already maps to an active requester lease.
  void process(const std::string & requester_identity, const SubscriptionHeartbeat & update);
  // Expires only the client session leases used for anonymous-heartbeat fallback.
  void pruneExpiredClientSessionLeases();

private:
  // Time-bound binding from the wire session_id to requester_identity.
  struct ClientSessionLease
  {
    std::string requester_identity;
    std::chrono::steady_clock::time_point expiry;
  };

  // Returns the requester identity that owns this heartbeat, binding or renewing the wire
  // session_id when present. A missing requester_identity is accepted only for a known,
  // unexpired client session lease.
  std::optional<std::string> resolveRequesterIdentity(
    const std::string & requester_identity,
    const SubscriptionHeartbeat & update,
    std::chrono::steady_clock::time_point requester_lease_expiry);
  // Renews a client session lease for exactly one requester identity until requester_lease_expiry.
  // Conflicts are rejected so a delayed or replayed heartbeat cannot steal another requester's
  // lease.
  bool renewClientSessionLease(
    const std::string & client_session_id,
    const std::string & requester_identity,
    std::chrono::steady_clock::time_point requester_lease_expiry);
  void publishSubscriptionStatus(
    const std::string & requester_identity,
    const std::optional<std::string> & client_session_id,
    std::chrono::steady_clock::time_point requester_lease_expiry,
    const nlohmann::json & streams);

  SubscriptionRegistry & subscription_registry_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  rclcpp::Clock::SharedPtr clock_;
  // Tracks which requester identity is allowed to renew a wire session_id anonymously until
  // expiry. A client session lease is not a room connection and not a participant_session.
  std::unordered_map<std::string, ClientSessionLease> client_session_leases_;
  EventThrottle client_session_conflict_throttle_{std::chrono::seconds(5)};
};

}  // namespace livekit_ros2_bridge
