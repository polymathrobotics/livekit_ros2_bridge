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
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "payloads/stream_control_payloads.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialized_message.hpp"
#include "video_config.hpp"

namespace livekit_ros2_bridge
{

class VideoSidecarSupervisor;

using SendCdrMessageFn =
  std::function<void(const std::string & track_name, const std::uint8_t * data, std::size_t size)>;

using PublishCdrTrackFn = std::function<void(const std::string & track_name, std::size_t generation)>;
using UnpublishCdrTrackFn = std::function<void(const std::string & track_name)>;

struct StreamUnavailableError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

enum class RequesterLeaseRemovalReason
{
  kParticipantDisconnected,
  kLeaseExpired
};

// Owns requester leases and the shared stream resources they keep alive.
class SubscriptionRegistry final
{
public:
  using Clock = std::chrono::steady_clock;

  SubscriptionRegistry(
    rclcpp::Node & node,
    SendCdrMessageFn send_cdr_fn,
    PublishCdrTrackFn publish_cdr_track_fn,
    UnpublishCdrTrackFn unpublish_cdr_track_fn,
    VideoSidecarSupervisor * video_sidecar_supervisor,
    const VideoConfig * video_config = nullptr);

  StreamStatus renewSubscription(
    const std::string & requester_identity, const SubscriptionRequest & entry, Clock::time_point expiry);
  StreamStatus renewSubscription(
    const std::string & requester_identity,
    const std::string & topic,
    int preferred_interval_ms,
    Clock::time_point expiry);

  // Participant disconnects do not clear leases immediately because a page refresh often rejoins
  // with the same requester identity. Instead we remember which identities need their shared CDR
  // publications replayed once the next heartbeat confirms they are back.
  void markRequesterForCdrReplay(const std::string & requester_identity, std::size_t generation);
  void replayCdrTracksForRequester(const std::string & requester_identity);
  void removeRequesterLeases(const std::string & requester_identity);
  void sweepExpiredLeases();
  bool hasSubscription(
    const std::string & resource, SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic) const;
  void resetSessionState();
  void shutdown();

  bool onCdrTrackPublished(const std::string & track_name, std::size_t generation);
  std::size_t registryGeneration() const;
  void onCdrTrackFailed(const std::string & track_name);

private:
  enum class CdrTrackState
  {
    kNone,
    kPending,
    kPublished,
    kFailed
  };

  struct RequesterLease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  struct DataTrackResource
  {
    std::shared_ptr<rclcpp::GenericSubscription> subscription_handle;
    std::optional<Clock::time_point> last_sent_time;
    std::string track_name;
    int applied_interval_ms = 0;
    CdrTrackState cdr_track_state = CdrTrackState::kNone;
    std::size_t generation = 0;
  };

  struct SubscriptionState
  {
    SubscriptionTargetKind target_kind = SubscriptionTargetKind::Topic;
    std::string resource;
    std::string interface_type;
    std::string source_kind;
    std::string ingest_mode;
    std::string selected_config_key;
    std::string degraded_reason;
    std::string video_publisher_identity;
    std::optional<SidecarLaunchSpec> sidecar_launch_spec;
    std::map<std::string, RequesterLease> requesters;
    std::optional<DataTrackResource> data_track_resource;

    DataTrackResource * data_track_ptr()
    {
      return data_track_resource ? &*data_track_resource : nullptr;
    }

    const DataTrackResource * data_track_ptr() const
    {
      return data_track_resource ? &*data_track_resource : nullptr;
    }
  };

  using SubscriptionStateMap = std::unordered_map<std::string, SubscriptionState>;
  using RequesterIdentityLeasePredicate =
    std::function<bool(const std::string & requester_identity, const RequesterLease &)>;

  static StreamStatus makeStreamStatus(const SubscriptionState & sub);
  static int computeAppliedIntervalMs(const std::map<std::string, RequesterLease> & requesters);
  static std::string deriveTrackName(const std::string & normalized_topic);
  static std::string makeSubscriptionKey(SubscriptionTargetKind target_kind, const std::string & resource);

  void refreshExistingLease(
    SubscriptionState & sub, const std::string & requester_identity, const RequesterLease & requester_lease);
  SubscriptionState createVideoSubscription(
    const SubscriptionRequest & entry,
    const std::string & interface_type,
    const std::string & requester_identity,
    const RequesterLease & requester_lease);
  SubscriptionState createDataSubscription(
    const SubscriptionRequest & entry,
    const std::string & interface_type,
    const std::string & subscription_key,
    const std::string & requester_identity,
    const RequesterLease & requester_lease);
  void assignVideoMetadata(
    SubscriptionState & sub, const SidecarLaunchSpec & sidecar_launch_spec, std::string publisher_identity);
  DataTrackResource createPendingDataTrackResource(
    const std::string & topic,
    const std::string & interface_type,
    const std::string & subscription_key,
    const std::map<std::string, RequesterLease> & requesters,
    const std::string & requester_identity);
  void publishPendingCdrTrack(
    const std::string & topic, DataTrackResource & data, const std::string & requester_identity);
  bool beginMessageCallback(std::size_t callback_generation);
  void endMessageCallback();
  std::size_t currentMessageCallbackGeneration() const;
  std::size_t quiesceMessageCallbacks();
  void resumeMessageCallbacks(std::size_t callback_generation);
  void removeRequesterLeasesIf(
    const RequesterIdentityLeasePredicate & should_remove,
    RequesterLeaseRemovalReason reason,
    Clock::time_point reference_time);
  SubscriptionStateMap::iterator findByTrackName(const std::string & track_name);
  SubscriptionStateMap::iterator pruneRequesterState(
    SubscriptionStateMap::iterator it, RequesterLeaseRemovalReason reason);
  void handleSerializedMessage(const std::string & topic, const rclcpp::SerializedMessage & message);
  bool shouldSkipDueToInterval(DataTrackResource & resource);
  void destroyResource(SubscriptionState & sub);
  void clearSubscriptions();

  rclcpp::Node & node_;
  SendCdrMessageFn send_cdr_fn_;
  PublishCdrTrackFn publish_cdr_track_fn_;
  UnpublishCdrTrackFn unpublish_cdr_track_fn_;
  VideoSidecarSupervisor * video_sidecar_supervisor_;
  VideoConfig default_video_config_;
  const VideoConfig * video_config_;
  mutable std::mutex message_callback_mutex_;
  std::condition_variable message_callback_quiesced_;
  bool message_callbacks_enabled_ = true;
  std::size_t active_message_callbacks_ = 0U;
  std::size_t message_callback_generation_ = 0U;
  std::atomic<bool> is_shutdown_{false};
  std::atomic<std::size_t> registry_generation_{0};
  SubscriptionStateMap subscriptions_;
  std::unordered_set<std::string> requesters_needing_cdr_replay_;
};

}  // namespace livekit_ros2_bridge
