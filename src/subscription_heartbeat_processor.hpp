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
#include "core/subscriptions.hpp"
#include "rclcpp/clock.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
class SubscriptionRegistry;

// Resolves heartbeat lease, renews subscriptions, and publishes subscription-status responses. This
// type has no internal synchronization; callers are expected to serialize `process()` and
// `pruneExpiredSessionLeases()` on one execution lane.
class SubscriptionHeartbeatProcessor final
{
public:
  SubscriptionHeartbeatProcessor(
    SubscriptionRegistry & subscription_registry,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock);

  void process(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);
  // Expires the client-session leases. There is no background timer; callers choose when this
  // maintenance runs.
  void pruneExpiredSessionLeases();

private:
  struct SessionLease
  {
    std::string requester_identity;
    std::chrono::steady_clock::time_point expiry;
  };

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
