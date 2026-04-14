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
#include <vector>

#include "access_policy.hpp"
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
  struct ResolvedLease
  {
    std::string requester_identity;
    std::optional<std::string> session_id;
    std::chrono::steady_clock::time_point expiry;
  };

  static constexpr auto kLogThrottle = std::chrono::seconds(5);

  SubscriptionRegistry & subscription_registry_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  // Used only for throttled ROS logging. Lease ownership uses `steady_clock` so ROS/system time
  // jumps do not change anonymous-heartbeat acceptance.
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, SessionLease> leases_;
  EventThrottle conflict_throttle_{kLogThrottle};

  std::optional<ResolvedLease> resolveLease(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  SubscriptionReportedStatus renewSubscription(const ResolvedLease & lease, const SubscriptionDemand & demand);
  void publishStatuses(const ResolvedLease & lease, const std::vector<SubscriptionReportedStatus> & statuses);
};

}  // namespace livekit_ros2_bridge
