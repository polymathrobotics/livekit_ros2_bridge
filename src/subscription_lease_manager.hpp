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

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "access_policy.hpp"
#include "core/subscriptions.hpp"
#include "data_stream_registry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/node.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
class VideoStreamRegistry;

// Maps heartbeat-driven subscription demands onto shared data/video runtimes, tracks requester
// leases, and publishes subscription status back to the requester.
class SubscriptionLeaseManager final
{
  struct SharedSubscription;

public:
  using Clock = std::chrono::steady_clock;

  SubscriptionLeaseManager(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    rclcpp::Clock::SharedPtr clock,
    DataStreamRegistry & data_stream_registry,
    VideoStreamRegistry & video_stream_registry,
    Clock::duration heartbeat_lease_duration = std::chrono::seconds(45));

  void handleHeartbeat(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);

  // Participant disconnects can leave a rejoined requester unable to see an already published
  // data track. If the requester still owns any published data subscription, mark it for one
  // republish on the next heartbeat-confirmed reconnect.
  void onRemoteParticipantDisconnected(const std::string & requester_identity);
  void pruneExpiredLeases();
  SharedSubscription * findSubscription(SubscriptionTargetKind kind, const std::string & name);
  const SharedSubscription * findSubscription(SubscriptionTargetKind kind, const std::string & name) const;
  void resetSessionState();
  void shutdown();

private:
  struct SessionLease
  {
    std::string requester_identity;
    Clock::time_point expiry;
  };

  struct Lease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  struct SharedSubscription
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string interface_type;
    SubscriptionDeliveryKind delivery_kind = SubscriptionDeliveryKind::kData;
    std::map<std::string, Lease> leases;
  };

  using SubscriptionMap = std::unordered_map<std::string, SharedSubscription>;
  using LeasePredicate = std::function<bool(const std::string & requester_identity, const Lease &)>;

  static constexpr auto kLogThrottle = std::chrono::seconds(5);
  static int appliedIntervalMs(const std::map<std::string, Lease> & leases);

  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  rclcpp::Clock::SharedPtr clock_;
  DataStreamRegistry & data_stream_registry_;
  VideoStreamRegistry & video_stream_registry_;
  Clock::duration heartbeat_lease_duration_;

  std::atomic<bool> is_shutdown_{false};
  std::unordered_map<std::string, SessionLease> session_leases_;
  SubscriptionMap subscriptions_;
  std::unordered_set<std::string> republish_requesters_;
  EventThrottle conflict_throttle_{kLogThrottle};

  std::optional<std::string> resolveRequesterIdentity(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  SubscriptionStatus createSubscription(
    const SubscriptionDemand & demand, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus renewExistingSubscription(
    SharedSubscription & subscription, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus renewSubscription(
    const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry);
  SubscriptionStatus statusFor(const SharedSubscription & subscription) const;

  void destroyRuntime(SharedSubscription & subscription, bool log_destroy = true);
  void removeLeasesIf(const LeasePredicate & should_remove, Clock::time_point reference_time);
  void republishDataTracks(const std::string & requester_identity);
};

}  // namespace livekit_ros2_bridge
