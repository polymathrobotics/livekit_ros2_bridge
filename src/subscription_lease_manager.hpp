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
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
class VideoStreamRegistry;

struct StreamUnavailableError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

enum class LeaseRemovalReason
{
  kParticipantDisconnected,
  kLeaseExpired
};

// Owns heartbeat session recovery plus requester leases and shared subscription coordination
// across data and video. Multiple heartbeat demands for one canonical target collapse into one
// shared runtime and one applied delivery state.
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
    VideoStreamRegistry * video_stream_registry,
    const VideoStreamConfig * video_stream_config = nullptr);

  void handleHeartbeat(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);

  // Refreshes or creates the shared canonical subscription for this target. `demand.name` is
  // expected to already be canonical and non-empty. Multiple requesters for the same normalized
  // target collapse into one runtime; renewing updates only that requester's lease state and may
  // restart a failed publication.
  SubscriptionStatus renewSubscription(
    const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry);
  SubscriptionStatus renewSubscription(
    const std::string & requester_identity,
    const std::string & topic,
    int preferred_interval_ms,
    Clock::time_point expiry);

  // Participant disconnects can leave a rejoined requester unable to see an already published
  // data track. If the requester still owns any published data subscription, mark it for one
  // republish on the next heartbeat-confirmed reconnect.
  void onRemoteParticipantDisconnected(const std::string & requester_identity);
  // Republishes currently published data tracks for a requester once a fresh heartbeat proves the
  // requester has rejoined and still owns those subscriptions.
  void republishDataTracks(const std::string & requester_identity);
  // Removes one requester's lease from every shared subscription. Targets still owned by other
  // requesters stay alive; any queued data-track republish hint for this requester is cleared.
  void revokeRequesterLeases(const std::string & requester_identity);
  // Sweeps session and target leases whose expiry has passed. Shared subscriptions survive while
  // at least one requester remains, and data targets recompute their merged delivery interval from
  // the survivors.
  void pruneExpiredLeases();
  SharedSubscription * findSubscription(SubscriptionTargetKind kind, const std::string & name);
  const SharedSubscription * findSubscription(SubscriptionTargetKind kind, const std::string & name) const;
  // Session-scoped teardown for room reconnects. Clears current subscriptions and tears down the
  // active data/video runtimes for the old room session.
  void resetSessionState();
  // Terminal teardown. After shutdown later renewals fail with StreamUnavailableError.
  void shutdown();

private:
  struct SessionLease
  {
    std::string requester_identity;
    Clock::time_point expiry;
  };

  struct ResolvedLease
  {
    std::string requester_identity;
    std::optional<std::string> session_id;
    Clock::time_point expiry;
  };

  struct Lease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  struct VideoStreamHandle
  {
    std::string track_name;
    VideoStreamSpec stream_spec;
  };

  struct SharedSubscription
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string interface_type;
    std::map<std::string, Lease> leases;
    std::optional<VideoStreamHandle> video;
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
  // Optional non-owning dependency. Null means video subscriptions cannot be started.
  VideoStreamRegistry * video_stream_registry_;
  VideoStreamConfig default_video_stream_config_;
  // Non-owning pointer that always references either the caller-supplied config or the in-object
  // default above.
  const VideoStreamConfig * video_stream_config_;

  std::atomic<bool> is_shutdown_{false};
  std::unordered_map<std::string, SessionLease> session_leases_;
  SubscriptionMap subscriptions_;

  // Requesters whose next confirmed heartbeat should force currently published data tracks
  // through an unpublish/publish cycle so the rejoined participant session sees them again.
  std::unordered_set<std::string> republish_requesters_;
  EventThrottle conflict_throttle_{kLogThrottle};

  std::optional<ResolvedLease> resolveLease(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  SubscriptionReportedStatus renewHeartbeatSubscription(const ResolvedLease & lease, const SubscriptionDemand & demand);
  void publishStatuses(const ResolvedLease & lease, const std::vector<SubscriptionReportedStatus> & statuses);
  SubscriptionStatus statusFor(const SharedSubscription & subscription) const;
  VideoStreamRegistry & videoRegistry() const;
  SubscriptionStatus renewExistingSubscription(
    SharedSubscription & subscription, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus createSubscription(
    const SubscriptionDemand & demand, const std::string & requester_identity, const Lease & lease);
  void removeLeasesIf(
    const LeasePredicate & should_remove, LeaseRemovalReason reason, Clock::time_point reference_time);
  void destroyRuntime(SharedSubscription & subscription, bool log_destroy = true);
};

}  // namespace livekit_ros2_bridge
