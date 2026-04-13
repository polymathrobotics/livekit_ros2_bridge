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
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "data_stream_instance.hpp"
#include "rclcpp/node.hpp"
#include "subscription_qos.hpp"
#include "subscription_types.hpp"
#include "utils/quiesce_gate.hpp"
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

// Owns requester leases and shared subscription coordination across data and video. Multiple
// heartbeat demands for one canonical target collapse into one shared runtime and one applied
// delivery state.
class SubscriptionRegistry final
{
public:
  using Clock = std::chrono::steady_clock;

  SubscriptionRegistry(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    VideoStreamRegistry * video_stream_registry,
    const VideoStreamConfig * video_stream_config = nullptr,
    const SubscriptionQosConfig * subscription_qos_config = nullptr);

  // Refreshes or creates the shared canonical subscription for this target. Multiple requesters
  // for the same normalized target collapse into one runtime; renewing updates only that
  // requester's lease state and may restart a failed publication.
  SubscriptionStatus renewSubscription(
    const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry);
  SubscriptionStatus renewSubscription(
    const std::string & requester_identity,
    const std::string & topic,
    int preferred_interval_ms,
    Clock::time_point expiry);

  // Participant disconnect callbacks can lag behind lease expiry or a same-topic resubscribe. The
  // caller supplies the observed generation for that session; only a matching generation queues
  // the requester for data-track republish on the next heartbeat-confirmed reconnect.
  void queueDataTrackRepublish(const std::string & requester_identity, std::size_t generation);
  // Republishes currently published data tracks for a requester once a fresh heartbeat proves the
  // requester has rejoined and still owns those subscriptions.
  void republishDataTracks(const std::string & requester_identity);
  // Removes one requester's lease from every shared subscription. Targets still owned by other
  // requesters stay alive; any queued data-track republish hint for this requester is cleared.
  void revokeRequesterLeases(const std::string & requester_identity);
  // Sweeps leases whose expiry has passed. Shared entries survive while at least one requester
  // remains, and data targets recompute their merged delivery interval from the survivors.
  void pruneExpiredLeases();
  bool hasSubscription(
    const std::string & resource, SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic) const;
  // Session-scoped teardown for room reconnects. Clears current subscriptions, advances lifetime
  // generations, and then re-opens the callback gate for the next session.
  void resetSessionState();
  // Terminal teardown. After shutdown the callback gate stays closed and later renewals fail with
  // StreamUnavailableError.
  void shutdown();

  // DataTrackPublisher completion path. The publish generation identifies the attempt that
  // reserved the deterministic track name, so late callbacks from an older lifetime can be
  // rejected instead of reviving a replacement subscription entry.
  bool onDataTrackPublished(const std::string & track_name, std::size_t generation);
  // Snapshot token for callers that need to tie later disconnect handling to the currently
  // observed subscription set.
  std::size_t generation() const;
  void onDataTrackFailed(const std::string & track_name);

private:
  struct Lease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  struct VideoRuntime
  {
    std::string track_name;
    VideoStreamSpec stream_spec;
  };

  struct Entry
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string interface_type;
    std::map<std::string, Lease> leases;
    // Exactly one runtime exists per canonical target. Data targets keep their shared
    // DataStreamInstance here; video targets keep the external video runtime state needed to stop
    // or restart the publication without a DataStreamInstance.
    std::variant<std::shared_ptr<DataStreamInstance>, VideoRuntime> runtime = std::shared_ptr<DataStreamInstance>{};
  };

  using EntryMap = std::unordered_map<std::string, Entry>;
  using LeasePredicate = std::function<bool(const std::string & requester_identity, const Lease &)>;

  static SubscriptionStatus statusFor(const Entry & entry);
  static int appliedIntervalMs(const std::map<std::string, Lease> & leases);
  static std::string keyFor(SubscriptionTargetKind target_kind, const std::string & resource);

  VideoStreamRegistry & videoRegistry() const;
  static DataStreamInstance * dataStream(Entry & entry);
  static const DataStreamInstance * dataStream(const Entry & entry);
  void removeLeasesIf(
    const LeasePredicate & should_remove, LeaseRemovalReason reason, Clock::time_point reference_time);
  EntryMap::iterator findDataByTrackName(const std::string & track_name);
  void destroyRuntime(Entry & entry);
  void clearSubscriptions();

  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  // Optional non-owning dependency. Null means video subscriptions cannot be started.
  VideoStreamRegistry * video_stream_registry_;
  VideoStreamConfig default_video_stream_config_;
  // Non-owning pointer that always references either the caller-supplied config or the in-object
  // default above.
  const VideoStreamConfig * video_stream_config_;
  const SubscriptionQosConfig * subscription_qos_config_;

  // Subscriptions capture the gate's current generation in their message callback.
  // Reset/shutdown quiesce and advance that generation before teardown so queued callbacks from
  // the old session self-reject on entry.
  QuiesceGate message_callback_gate_;
  std::atomic<bool> is_shutdown_{false};
  // Advanced whenever a deterministic data-track identity could be reused by a replacement
  // subscription or after a full registry reset. External lifecycle callbacks must match this
  // snapshot before acting on remembered requester state.
  std::atomic<std::size_t> registry_generation_{0};
  EntryMap subscriptions_;

  // Requesters whose next confirmed heartbeat should force currently published data tracks
  // through an unpublish/publish cycle so the rejoined participant session sees them again.
  std::unordered_set<std::string> republish_requesters_;
};

}  // namespace livekit_ros2_bridge
