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
#include "payloads/stream_control_payloads.hpp"
#include "rclcpp/clock.hpp"

namespace livekit_ros2_bridge
{

class RoomSession;
class SubscriptionRegistry;

class SubscriptionHeartbeatProcessor final
{
public:
  SubscriptionHeartbeatProcessor(
    SubscriptionRegistry & subscription_registry,
    RoomSession & session,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock);

  void process(const std::string & requester_identity, const SubscriptionHeartbeat & update);
  void sweepExpiredSessionLeases();

private:
  struct SessionLease
  {
    std::string requester_identity;
    std::chrono::steady_clock::time_point expiry;
  };

  std::optional<std::string> resolveRequesterIdentity(
    const std::string & requester_identity,
    const SubscriptionHeartbeat & update,
    std::chrono::steady_clock::time_point expiry);
  bool bindSessionId(
    const std::string & session_id,
    const std::string & requester_identity,
    std::chrono::steady_clock::time_point expiry);
  void publishSubscriptionStatus(
    const std::string & requester_identity,
    const std::optional<std::string> & session_id,
    std::chrono::steady_clock::time_point expiry,
    const nlohmann::json & streams);

  SubscriptionRegistry & subscription_registry_;
  RoomSession & room_session_;
  AccessPolicy access_policy_;
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, SessionLease> session_leases_;
};

}  // namespace livekit_ros2_bridge
