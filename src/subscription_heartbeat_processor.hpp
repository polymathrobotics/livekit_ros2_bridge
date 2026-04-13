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
#include <string>
#include <unordered_map>

#include "access_policy.hpp"
#include "nlohmann/json_fwd.hpp"
#include "rclcpp/clock.hpp"
#include "subscription_types.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
class SubscriptionRegistry;

// Resolves heartbeat lease ownership, including anonymous renewals via a still-leased wire
// `session_id`, and publishes subscription-status responses.
// This type has no internal synchronization; callers are expected to serialize `process()` and
// `pruneExpiredLeases()` on one execution lane.
class SubscriptionHeartbeatProcessor final
{
public:
  SubscriptionHeartbeatProcessor(
    SubscriptionRegistry & subscription_registry,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock);

  // Renews every demand for the resolved requester identity. An empty `requester_identity` is
  // treated as anonymous and succeeds only when `heartbeat.session_id` already owns a live lease.
  void process(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);
  // Expires the client-session leases used for anonymous-heartbeat fallback. There is no
  // background timer; callers choose when this maintenance runs.
  void pruneExpiredLeases();

private:
  // Tracks the requester identity bound to a wire `session_id` so anonymous heartbeats from an
  // already authenticated browser tab can renew until expiry.
  struct SessionLease
  {
    std::string requester_identity;
    std::chrono::steady_clock::time_point expiry;
  };

  // Accepted-heartbeat processing keeps one resolved requester identity and one shared lease
  // expiry across both subscription renewal and the echoed status envelope.
  struct ResolvedHeartbeatLease
  {
    std::string requester_identity;
    std::optional<std::string> session_id;
    std::chrono::steady_clock::time_point expiry;
  };

  std::optional<ResolvedHeartbeatLease> resolveHeartbeatLease(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  // Recovers the requester identity for an anonymous heartbeat from an existing leased
  // `session_id`, and extends that lease when the fallback succeeds.
  std::optional<std::string> resolveAnonymousIdentity(
    const std::optional<std::string> & session_id, std::chrono::steady_clock::time_point lease_expiry);
  nlohmann::json renewSubscriptionStatuses(
    const ResolvedHeartbeatLease & lease, const std::vector<SubscriptionDemand> & subscriptions);

  void publishStatuses(const ResolvedHeartbeatLease & lease, const nlohmann::json & statuses);

  SubscriptionRegistry & subscription_registry_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  // Used only for throttled ROS logging. Lease ownership uses `steady_clock` so ROS/system time
  // jumps do not change anonymous-heartbeat acceptance.
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, SessionLease> session_leases_;
  EventThrottle session_conflict_throttle_{std::chrono::seconds(5)};
};

}  // namespace livekit_ros2_bridge
