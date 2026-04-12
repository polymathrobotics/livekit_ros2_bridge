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
#include "payloads/stream_control_payloads.hpp"
#include "rclcpp/node.hpp"
#include "subscription_qos.hpp"
#include "utils/quiesce_guard.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
class VideoStreamRegistry;

struct StreamUnavailableError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

enum class RequesterLeaseRemovalReason
{
  kParticipantDisconnected,
  kLeaseExpired
};

// Owns requester leases and shared subscription coordination across data and video.
// For data topics, the registry owns one DataStreamInstance per shared topic lease set, and each
// instance owns its one DataTrackPublisher for the LiveKit publication.
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

  StreamStatus renewSubscription(
    const std::string & requester_identity, const SubscriptionRequest & entry, Clock::time_point expiry);
  StreamStatus renewSubscription(
    const std::string & requester_identity,
    const std::string & topic,
    int preferred_interval_ms,
    Clock::time_point expiry);

  // Participant disconnect callbacks can lag behind lease expiry or a same-topic resubscribe. The
  // caller supplies the registry generation observed for that session; only a matching generation
  // marks the requester identity for data-track republish on the next heartbeat-confirmed
  // reconnect.
  void markRequesterForDataTrackRepublish(const std::string & requester_identity, std::size_t generation);
  // Republishes currently published data tracks for a requester once a fresh heartbeat proves the
  // requester has rejoined and still owns those subscriptions.
  void republishDataTracksForRequester(const std::string & requester_identity);
  void revokeRequesterLeases(const std::string & requester_identity);
  void pruneExpiredLeases();
  bool hasSubscription(
    const std::string & resource, SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic) const;
  void resetSessionState();
  void shutdown();

  bool onDataTrackPublished(const std::string & track_name, std::size_t generation);
  std::size_t registryGeneration() const;
  void onDataTrackFailed(const std::string & track_name);

private:
  struct RequesterLease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  struct VideoTrackResource
  {
    std::string track_name;
    VideoStreamSpec stream_spec;
  };

  struct SubscriptionState
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string resource;
    std::string interface_type;
    std::map<std::string, RequesterLease> requesters;
    std::variant<std::shared_ptr<DataStreamInstance>, VideoTrackResource> resource_state =
      std::shared_ptr<DataStreamInstance>{};
  };

  using SubscriptionStateMap = std::unordered_map<std::string, SubscriptionState>;
  using RequesterIdentityLeasePredicate =
    std::function<bool(const std::string & requester_identity, const RequesterLease &)>;

  static StreamStatus makeStreamStatus(const SubscriptionState & sub);
  static int computeAppliedIntervalMs(const std::map<std::string, RequesterLease> & requesters);
  static std::string makeSubscriptionKey(SubscriptionTargetKind target_kind, const std::string & resource);

  void renewExistingLease(
    SubscriptionState & sub, const std::string & requester_identity, const RequesterLease & requester_lease);
  SubscriptionState createVideoSubscription(
    const SubscriptionRequest & entry,
    const std::string & interface_type,
    const std::string & requester_identity,
    const RequesterLease & requester_lease);
  SubscriptionState createDataSubscription(
    const SubscriptionRequest & entry,
    const std::string & interface_type,
    const std::string & requester_identity,
    const RequesterLease & requester_lease);
  void assignVideoMetadata(SubscriptionState & sub, VideoStreamSpec stream_spec, std::string track_name);
  std::shared_ptr<DataStreamInstance> createDataStreamInstance(
    const std::string & topic,
    const std::string & interface_type,
    const std::map<std::string, RequesterLease> & requesters);
  VideoStreamRegistry & videoStreamRegistry() const;
  const VideoStreamSpec & videoStreamSpec(const SubscriptionState & sub) const;
  static DataStreamInstance * dataStreamInstance(SubscriptionState & sub);
  static const DataStreamInstance * dataStreamInstance(const SubscriptionState & sub);
  void revokeRequesterLeasesIf(
    const RequesterIdentityLeasePredicate & should_remove,
    RequesterLeaseRemovalReason reason,
    Clock::time_point reference_time);
  SubscriptionStateMap::iterator findByTrackName(const std::string & track_name);
  SubscriptionStateMap::iterator pruneRequesterState(
    SubscriptionStateMap::iterator it, RequesterLeaseRemovalReason reason);
  void destroyResource(SubscriptionState & sub);
  void clearSubscriptions();

  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  VideoStreamRegistry * video_stream_registry_;
  VideoStreamConfig default_video_stream_config_;
  const VideoStreamConfig * video_stream_config_;
  const SubscriptionQosConfig * subscription_qos_config_;
  // Subscriptions capture the gate's current generation in their message callback.
  // Reset/shutdown quiesce and advance that generation before teardown so queued callbacks from
  // the old session self-reject on entry.
  QuiesceGate message_callback_gate_;
  std::atomic<bool> is_shutdown_{false};
  std::atomic<std::size_t> registry_generation_{0};
  SubscriptionStateMap subscriptions_;
  // Requesters whose next confirmed heartbeat should force currently published data tracks
  // through an unpublish/publish cycle so the rejoined participant session sees them again.
  std::unordered_set<std::string> requesters_needing_data_track_republish_;
};

}  // namespace livekit_ros2_bridge
