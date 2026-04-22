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
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "access_policy.hpp"
#include "core/subscriptions.hpp"
#include "rclcpp/node.hpp"
#include "ros_node_interfaces.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class DataTrackPublisher;
class RoomConnection;
struct SubscriptionQosConfig;
class VideoStreamRegistry;

// Maps heartbeat-driven subscription demands onto shared data/video runtimes, tracks requester
// leases, and publishes subscription status back to the requester.
class SubscriptionLeaseManager final
{
  struct Subscription;

public:
  using Clock = std::chrono::steady_clock;

  SubscriptionLeaseManager(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    VideoStreamRegistry & video_stream_registry,
    const SubscriptionQosConfig * qos_config = nullptr,
    Clock::duration heartbeat_lease_duration = std::chrono::seconds(45));

  SubscriptionLeaseManager(
    SubscriptionNodeInterfaces interfaces,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    VideoStreamRegistry & video_stream_registry,
    const SubscriptionQosConfig * qos_config = nullptr,
    Clock::duration heartbeat_lease_duration = std::chrono::seconds(45));
  ~SubscriptionLeaseManager();

  void handleHeartbeat(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);

  // Participant disconnects can leave a rejoined requester unable to see an already published
  // data track. If the requester still owns any published data subscription, mark it for one
  // republish on the next heartbeat-confirmed reconnect.
  void onRemoteParticipantDisconnected(const std::string & requester_identity);
  void pruneExpiredLeases();
  Subscription * find(SubscriptionTargetKind kind, const std::string & name);
  const Subscription * find(SubscriptionTargetKind kind, const std::string & name) const;
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

  struct Subscription
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string interface_type;
    std::map<std::string, Lease> leases;
    std::shared_ptr<DataTrackPublisher> data_publisher;
  };

  struct ExpiredLeaseRemoval
  {
    std::string requester_identity;
    Clock::time_point expiry;
  };

  using SubscriptionMap = std::unordered_map<std::string, Subscription>;

  static constexpr auto kLogThrottle = std::chrono::seconds(5);
  static int appliedIntervalMs(const std::map<std::string, Lease> & leases);
  static std::vector<ExpiredLeaseRemoval> collectExpiredLeaseRemovals(
    const Subscription & subscription, Clock::time_point reference_time);
  static bool isVideoSubscription(const Subscription & subscription);

  SubscriptionNodeInterfaces interfaces_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  VideoStreamRegistry & video_stream_registry_;
  const SubscriptionQosConfig * qos_config_;
  Clock::duration heartbeat_lease_duration_;

  std::atomic<bool> is_shutdown_{false};
  std::unordered_map<std::string, SessionLease> session_leases_;
  SubscriptionMap subscriptions_;
  std::unordered_set<std::string> republish_requesters_;
  EventThrottle conflict_throttle_{kLogThrottle};

  std::optional<std::string> resolveIdentity(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  DataTrackPublisher & requireDataPublisher(const Subscription & subscription) const;
  SubscriptionStatus create(
    const SubscriptionDemand & demand, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus renew(Subscription & subscription, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus ensure(
    const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry);
  SubscriptionStatus status(const Subscription & subscription) const;

  void applyExpiredLeaseRemovals(
    Subscription & subscription, const std::vector<ExpiredLeaseRemoval> & removals, Clock::time_point reference_time);
  void destroy(Subscription & subscription, bool log_destroy = true);
  void pruneExpiredSubscriptionLeases(Clock::time_point reference_time);
  void refreshDataSubscriptionInterval(const Subscription & subscription);
  void republishTracks(const std::string & requester_identity);
};

}  // namespace livekit_ros2_bridge
