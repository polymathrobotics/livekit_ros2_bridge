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

#include "subscription_registry.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "utils/interface_types.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/scope_exit.hpp"
#include "video_sidecar_supervisor.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kDataSubscriptionDepth = 10U;
const auto kSubscriptionRegistryLogger = rclcpp::get_logger("subscription_registry");

const char * streamDeliveryKindString(StreamDeliveryKind delivery_kind)
{
  switch (delivery_kind) {
    case StreamDeliveryKind::kDataTrack:
      return protocol::kDeliveryKindDataTrack;
    case StreamDeliveryKind::kVideo:
      return protocol::kDeliveryKindVideo;
  }

  throw std::invalid_argument("stream delivery kind is invalid");
}

const std::string & requireRequesterIdentity(const std::string & requester_identity)
{
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }

  return requester_identity;
}

int sanitizePreferredIntervalMs(int preferred_interval_ms)
{
  return std::max(preferred_interval_ms, 0);
}

const char * requesterRemovalReasonToString(RequesterLeaseRemovalReason reason)
{
  switch (reason) {
    case RequesterLeaseRemovalReason::kParticipantDisconnected:
      return "participant_disconnected";
    case RequesterLeaseRemovalReason::kLeaseExpired:
      return "lease_expired";
  }

  return "unknown";
}

const char * subscriptionKindToString(SubscriptionTargetKind target_kind)
{
  return target_kind == SubscriptionTargetKind::Topic ? "topic" : "external";
}

SubscriptionRequest normalizeSubscriptionRequest(const SubscriptionRequest & entry)
{
  const auto & target = entry.target;
  const std::string normalized = target.kind == SubscriptionTargetKind::Topic ? normalizeRosResourceName(target.name)
                                                                              : normalizeExternalName(target.name);
  if (normalized.empty()) {
    throw std::invalid_argument(
      target.kind == SubscriptionTargetKind::Topic
        ? "heartbeat subscription target name must normalize to a non-empty topic name"
        : "heartbeat subscription target name must normalize to a non-empty external name");
  }

  return SubscriptionRequest{{target.kind, normalized}, entry.preferred_interval_ms};
}

std::string ensureVideoSidecar(VideoSidecarSupervisor & video_sidecar_supervisor, const SidecarLaunchSpec & spec)
{
  try {
    return video_sidecar_supervisor.ensureSidecar(spec);
  } catch (const std::exception & exc) {
    throw StreamUnavailableError(exc.what());
  }
}

}  // namespace

SubscriptionRegistry::SubscriptionRegistry(
  rclcpp::Node & node,
  SendCdrMessageFn send_cdr_fn,
  PublishCdrTrackFn publish_cdr_track_fn,
  UnpublishCdrTrackFn unpublish_cdr_track_fn,
  VideoSidecarSupervisor * video_sidecar_supervisor,
  const VideoConfig * video_config)
: node_(node)
, send_cdr_fn_(std::move(send_cdr_fn))
, publish_cdr_track_fn_(std::move(publish_cdr_track_fn))
, unpublish_cdr_track_fn_(std::move(unpublish_cdr_track_fn))
, video_sidecar_supervisor_(video_sidecar_supervisor)
, default_video_config_(makeDefaultVideoConfig())
, video_config_(video_config == nullptr ? &default_video_config_ : video_config)
{}

StreamStatus SubscriptionRegistry::renewSubscription(
  const std::string & requester_identity, const SubscriptionRequest & entry, Clock::time_point expiry)
{
  requireRequesterIdentity(requester_identity);
  const SubscriptionRequest normalized = normalizeSubscriptionRequest(entry);
  const auto & target = normalized.target;
  if (is_shutdown_.load()) {
    throw StreamUnavailableError("Subscription registry is shut down.");
  }

  const int preferred_interval_ms = sanitizePreferredIntervalMs(normalized.preferred_interval_ms.value_or(0));
  const std::string subscription_key = makeSubscriptionKey(target.kind, target.name);
  const RequesterLease requester_lease{preferred_interval_ms, expiry};

  auto it = subscriptions_.find(subscription_key);
  if (it != subscriptions_.end()) {
    refreshExistingLease(it->second, requester_identity, requester_lease);
    return makeStreamStatus(it->second);
  }

  SubscriptionState sub;
  if (target.kind == SubscriptionTargetKind::External) {
    sub = createVideoSubscription(normalized, "", requester_identity, requester_lease);
  } else {
    const std::string interface_type =
      requireUniqueInterfaceType(node_.get_topic_names_and_types(), target.name, "topic");
    if (classifyRosVideoInterfaceType(interface_type).has_value()) {
      sub = createVideoSubscription(normalized, interface_type, requester_identity, requester_lease);
    } else {
      sub = createDataSubscription(normalized, interface_type, requester_identity, requester_lease);
    }
  }

  StreamStatus stream_status = makeStreamStatus(sub);
  RCLCPP_INFO(
    kSubscriptionRegistryLogger,
    "event=subscription_created resource=%s kind=%s delivery=%s requester=%s",
    sub.resource.c_str(),
    subscriptionKindToString(sub.target_kind),
    streamDeliveryKindString(stream_status.delivery_kind),
    requester_identity.c_str());
  subscriptions_.emplace(subscription_key, std::move(sub));
  return stream_status;
}

StreamStatus SubscriptionRegistry::renewSubscription(
  const std::string & requester_identity,
  const std::string & topic,
  int preferred_interval_ms,
  Clock::time_point expiry)
{
  return renewSubscription(
    requester_identity, SubscriptionRequest{{SubscriptionTargetKind::Topic, topic}, preferred_interval_ms}, expiry);
}

void SubscriptionRegistry::markRequesterForCdrReplay(const std::string & requester_identity, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);
  if (generation != registry_generation_.load()) {
    return;
  }

  for (const auto & [subscription_key, sub] : subscriptions_) {
    (void)subscription_key;
    const auto * data = sub.data_track_ptr();
    if (data == nullptr || data->cdr_track_state != CdrTrackState::kPublished) {
      continue;
    }
    if (sub.requesters.find(requester_identity) == sub.requesters.end()) {
      continue;
    }

    requesters_needing_cdr_replay_.insert(requester_identity);
    return;
  }
}

void SubscriptionRegistry::replayCdrTracksForRequester(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);
  if (requesters_needing_cdr_replay_.erase(requester_identity) == 0U) {
    return;
  }

  for (auto & [subscription_key, sub] : subscriptions_) {
    (void)subscription_key;
    auto * data = sub.data_track_ptr();
    if (data == nullptr || data->cdr_track_state != CdrTrackState::kPublished) {
      continue;
    }
    if (sub.requesters.find(requester_identity) == sub.requesters.end()) {
      continue;
    }

    RCLCPP_INFO(
      kSubscriptionRegistryLogger,
      "event=cdr_track_replay topic=%s track_name=%s requester=%s",
      sub.resource.c_str(),
      data->track_name.c_str(),
      requester_identity.c_str());
    unpublish_cdr_track_fn_(data->track_name);
    data->cdr_track_state = CdrTrackState::kNone;
    data->last_sent_time.reset();
    publishPendingCdrTrack(sub.resource, *data, requester_identity);
  }
}

void SubscriptionRegistry::refreshExistingLease(
  SubscriptionState & sub, const std::string & requester_identity, const RequesterLease & requester_lease)
{
  const bool requester_already_present = sub.requesters.find(requester_identity) != sub.requesters.end();
  auto updated_requesters = sub.requesters;
  updated_requesters[requester_identity] = requester_lease;

  if (auto * data = sub.data_track_ptr()) {
    sub.requesters = std::move(updated_requesters);
    data->applied_interval_ms = computeAppliedIntervalMs(sub.requesters);
    if (!requester_already_present && data->cdr_track_state == CdrTrackState::kPublished) {
      // A new requester can receive stream status before LiveKit surfaces the existing published
      // data track to that participant session, so queue one replay when the requester first joins.
      requesters_needing_cdr_replay_.insert(requester_identity);
    }
    if (data->cdr_track_state == CdrTrackState::kNone || data->cdr_track_state == CdrTrackState::kFailed) {
      publishPendingCdrTrack(sub.resource, *data, requester_identity);
    }
    return;
  }

  const std::string publisher_identity = ensureVideoSidecar(videoSidecarSupervisor(), videoSidecarLaunchSpec(sub));
  sub.requesters = std::move(updated_requesters);
  sub.video_publisher_identity = publisher_identity;
}

SubscriptionRegistry::SubscriptionState SubscriptionRegistry::createVideoSubscription(
  const SubscriptionRequest & entry,
  const std::string & interface_type,
  const std::string & requester_identity,
  const RequesterLease & requester_lease)
{
  const auto & target = entry.target;
  SubscriptionState sub;
  sub.target_kind = target.kind;
  sub.resource = target.name;
  sub.interface_type = interface_type;
  sub.requesters.emplace(requester_identity, requester_lease);

  const SidecarLaunchSpec sidecar_launch_spec =
    target.kind == SubscriptionTargetKind::External
      ? resolvePipelineVideoLaunchSpec(*video_config_, target.name)
      : resolveRosVideoLaunchSpec(*video_config_, target.name, interface_type);
  assignVideoMetadata(sub, sidecar_launch_spec, ensureVideoSidecar(videoSidecarSupervisor(), sidecar_launch_spec));
  return sub;
}

SubscriptionRegistry::SubscriptionState SubscriptionRegistry::createDataSubscription(
  const SubscriptionRequest & entry,
  const std::string & interface_type,
  const std::string & requester_identity,
  const RequesterLease & requester_lease)
{
  const auto & target = entry.target;
  SubscriptionState sub;
  sub.target_kind = target.kind;
  sub.resource = target.name;
  sub.interface_type = interface_type;
  sub.requesters.emplace(requester_identity, requester_lease);
  sub.data_track_resource =
    createPendingDataTrackResource(target.name, interface_type, sub.requesters, requester_identity);
  return sub;
}

void SubscriptionRegistry::assignVideoMetadata(
  SubscriptionState & sub, const SidecarLaunchSpec & sidecar_launch_spec, std::string publisher_identity)
{
  sub.source_kind = videoSourceKindToString(sidecar_launch_spec.source_kind);
  sub.ingest_mode = sidecar_launch_spec.ingest_mode;
  sub.selected_config_key = sidecar_launch_spec.selected_config_key;
  sub.video_publisher_identity = std::move(publisher_identity);
  sub.sidecar_launch_spec = sidecar_launch_spec;
}

SubscriptionRegistry::DataTrackResource SubscriptionRegistry::createPendingDataTrackResource(
  const std::string & topic,
  const std::string & interface_type,
  const std::map<std::string, RequesterLease> & requesters,
  const std::string & requester_identity)
{
  const rclcpp::QoS qos(kDataSubscriptionDepth);

  DataTrackResource data;
  const std::size_t callback_generation = currentMessageCallbackGeneration();
  data.track_name = deriveTrackName(topic);
  data.applied_interval_ms = computeAppliedIntervalMs(requesters);
  data.generation = registry_generation_.load();
  publishPendingCdrTrack(topic, data, requester_identity);

  data.subscription_handle = node_.create_generic_subscription(
    topic,
    interface_type,
    qos,
    [this, callback_generation, normalized_topic = topic](std::shared_ptr<rclcpp::SerializedMessage> message) {
      // Reset/shutdown bumps callback_generation before tearing down subscriptions. Any queued
      // callback from the old generation exits before touching cleared subscription state.
      if (message == nullptr) {
        return;
      }
      if (!beginMessageCallback(callback_generation)) {
        return;
      }
      ScopeExit finish_delivery([this]() { endMessageCallback(); });
      handleSerializedMessage(normalized_topic, *message);
    });
  return data;
}

void SubscriptionRegistry::publishPendingCdrTrack(
  const std::string & topic, DataTrackResource & data, const std::string & requester_identity)
{
  data.cdr_track_state = CdrTrackState::kPending;
  RCLCPP_INFO(
    kSubscriptionRegistryLogger,
    "event=cdr_track_pending topic=%s track_name=%s requester=%s",
    topic.c_str(),
    data.track_name.c_str(),
    requester_identity.c_str());
  publish_cdr_track_fn_(data.track_name, data.generation);
}

VideoSidecarSupervisor & SubscriptionRegistry::videoSidecarSupervisor() const
{
  if (video_sidecar_supervisor_ == nullptr) {
    throw StreamUnavailableError("Video sidecars require livekit.api_key and livekit.api_secret.");
  }

  return *video_sidecar_supervisor_;
}

const SidecarLaunchSpec & SubscriptionRegistry::videoSidecarLaunchSpec(const SubscriptionState & sub) const
{
  if (!sub.sidecar_launch_spec.has_value()) {
    throw std::logic_error("video subscription invariant violated: sidecar launch spec is required");
  }

  return *sub.sidecar_launch_spec;
}

void SubscriptionRegistry::removeRequesterLeases(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);

  removeRequesterLeasesIf(
    [&requester_identity](const std::string & candidate_requester_identity, const RequesterLease &) {
      return candidate_requester_identity == requester_identity;
    },
    RequesterLeaseRemovalReason::kParticipantDisconnected,
    Clock::now());
}

void SubscriptionRegistry::sweepExpiredLeases()
{
  if (is_shutdown_.load()) {
    return;
  }
  const auto now = Clock::now();
  removeRequesterLeasesIf(
    [now](const std::string &, const RequesterLease & requester_lease) { return now >= requester_lease.expiry; },
    RequesterLeaseRemovalReason::kLeaseExpired,
    now);

  if (video_sidecar_supervisor_ != nullptr) {
    video_sidecar_supervisor_->maintainSidecars();
  }
}

bool SubscriptionRegistry::hasSubscription(const std::string & resource, SubscriptionTargetKind target_kind) const
{
  const std::string normalized =
    target_kind == SubscriptionTargetKind::Topic ? normalizeRosResourceName(resource) : normalizeExternalName(resource);
  if (normalized.empty()) {
    return false;
  }
  return subscriptions_.find(makeSubscriptionKey(target_kind, normalized)) != subscriptions_.end();
}

void SubscriptionRegistry::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }
  const std::size_t callback_generation = quiesceMessageCallbacks();
  requesters_needing_cdr_replay_.clear();
  clearSubscriptions();
  resumeMessageCallbacks(callback_generation);
}

void SubscriptionRegistry::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  (void)quiesceMessageCallbacks();
  clearSubscriptions();
}

bool SubscriptionRegistry::onCdrTrackPublished(const std::string & track_name, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return false;
  }
  auto it = findByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return false;
  }
  auto & sub = it->second;
  auto * data = sub.data_track_ptr();
  if (data == nullptr || data->cdr_track_state != CdrTrackState::kPending) {
    return false;
  }
  if (data->generation != generation) {
    return false;
  }
  data->cdr_track_state = CdrTrackState::kPublished;
  RCLCPP_INFO(
    kSubscriptionRegistryLogger,
    "event=cdr_track_published topic=%s track_name=%s",
    sub.resource.c_str(),
    data->track_name.c_str());
  return true;
}

void SubscriptionRegistry::onCdrTrackFailed(const std::string & track_name)
{
  if (is_shutdown_.load()) {
    return;
  }
  auto it = findByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return;
  }
  auto & sub = it->second;
  auto * data = sub.data_track_ptr();
  if (data == nullptr) {
    return;
  }
  RCLCPP_WARN(
    kSubscriptionRegistryLogger,
    "event=cdr_track_publish_failed topic=%s track_name=%s",
    sub.resource.c_str(),
    data->track_name.c_str());
  data->cdr_track_state = CdrTrackState::kFailed;
}

StreamStatus SubscriptionRegistry::makeStreamStatus(const SubscriptionState & sub)
{
  StreamStatus stream_status;
  stream_status.target = {sub.target_kind, sub.resource};
  stream_status.interface_type = sub.interface_type;
  stream_status.degraded_reason = sub.degraded_reason;

  const auto * data = sub.data_track_ptr();
  if (data != nullptr) {
    stream_status.delivery_kind = StreamDeliveryKind::kDataTrack;
    if (data->cdr_track_state == CdrTrackState::kPending || data->cdr_track_state == CdrTrackState::kPublished) {
      stream_status.track_name = data->track_name;
    }
    stream_status.applied_interval_ms = data->applied_interval_ms;
  } else {
    stream_status.delivery_kind = StreamDeliveryKind::kVideo;
    stream_status.publisher_identity = sub.video_publisher_identity;
  }

  return stream_status;
}

int SubscriptionRegistry::computeAppliedIntervalMs(const std::map<std::string, RequesterLease> & requesters)
{
  if (requesters.empty()) {
    return 0;
  }

  int applied_interval_ms = requesters.begin()->second.preferred_interval_ms;
  for (const auto & [id, lease] : requesters) {
    (void)id;
    applied_interval_ms = std::min(applied_interval_ms, lease.preferred_interval_ms);
  }
  return applied_interval_ms;
}

std::string SubscriptionRegistry::deriveTrackName(const std::string & normalized_topic)
{
  std::string name = "ros.cdr";
  for (char ch : normalized_topic) {
    name.push_back(ch == '/' ? '.' : ch);
  }
  return name;
}

std::string SubscriptionRegistry::makeSubscriptionKey(SubscriptionTargetKind target_kind, const std::string & resource)
{
  return (target_kind == SubscriptionTargetKind::Topic ? "topic:" : "external:") + resource;
}

bool SubscriptionRegistry::beginMessageCallback(std::size_t callback_generation)
{
  std::lock_guard<std::mutex> lock(message_callback_mutex_);
  if (!message_callbacks_enabled_ || callback_generation != message_callback_generation_) {
    return false;
  }

  ++active_message_callbacks_;
  return true;
}

void SubscriptionRegistry::endMessageCallback()
{
  {
    std::lock_guard<std::mutex> lock(message_callback_mutex_);
    if (active_message_callbacks_ == 0U) {
      return;
    }
    --active_message_callbacks_;
  }

  message_callback_quiesced_.notify_all();
}

std::size_t SubscriptionRegistry::currentMessageCallbackGeneration() const
{
  std::lock_guard<std::mutex> lock(message_callback_mutex_);
  return message_callback_generation_;
}

std::size_t SubscriptionRegistry::quiesceMessageCallbacks()
{
  std::unique_lock<std::mutex> lock(message_callback_mutex_);
  message_callbacks_enabled_ = false;
  // Force queued callbacks to compare against a new generation, then wait until every callback
  // that already entered endMessageCallback() before reset/shutdown continues.
  ++message_callback_generation_;
  message_callback_quiesced_.wait(lock, [this]() { return active_message_callbacks_ == 0U; });
  return message_callback_generation_;
}

void SubscriptionRegistry::resumeMessageCallbacks(std::size_t callback_generation)
{
  std::lock_guard<std::mutex> lock(message_callback_mutex_);
  if (message_callback_generation_ != callback_generation) {
    return;
  }
  message_callbacks_enabled_ = true;
}

void SubscriptionRegistry::removeRequesterLeasesIf(
  const RequesterIdentityLeasePredicate & should_remove,
  RequesterLeaseRemovalReason reason,
  Clock::time_point reference_time)
{
  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto & sub = it->second;
    bool removed_any = false;

    for (auto req_it = sub.requesters.begin(); req_it != sub.requesters.end();) {
      const auto & requester_identity = req_it->first;
      const auto & requester_lease = req_it->second;
      if (!should_remove(requester_identity, requester_lease)) {
        ++req_it;
        continue;
      }

      const auto remaining_requesters = sub.requesters.size() - 1U;
      const auto delta_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(requester_lease.expiry - reference_time).count();
      const char * time_label =
        reason == RequesterLeaseRemovalReason::kLeaseExpired ? "expired_by_ms" : "expires_in_ms";
      const long time_value = reason == RequesterLeaseRemovalReason::kLeaseExpired ? static_cast<long>(-delta_ms)
                                                                                   : static_cast<long>(delta_ms);
      RCLCPP_INFO(
        kSubscriptionRegistryLogger,
        "event=requester_lease_removed resource=%s kind=%s requester=%s reason=%s preferred_interval_ms=%d "
        "remaining_requesters=%zu %s=%ld",
        sub.resource.c_str(),
        subscriptionKindToString(sub.target_kind),
        requester_identity.c_str(),
        requesterRemovalReasonToString(reason),
        requester_lease.preferred_interval_ms,
        remaining_requesters,
        time_label,
        time_value);

      removed_any = true;
      requesters_needing_cdr_replay_.erase(requester_identity);
      req_it = sub.requesters.erase(req_it);
    }

    if (!removed_any) {
      ++it;
      continue;
    }

    it = pruneRequesterState(it, reason);
  }
}

SubscriptionRegistry::SubscriptionStateMap::iterator SubscriptionRegistry::findByTrackName(
  const std::string & track_name)
{
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    auto & sub = it->second;
    auto * data = sub.data_track_ptr();
    if (data != nullptr && data->track_name == track_name) {
      return it;
    }
  }
  return subscriptions_.end();
}

SubscriptionRegistry::SubscriptionStateMap::iterator SubscriptionRegistry::pruneRequesterState(
  SubscriptionStateMap::iterator it, RequesterLeaseRemovalReason reason)
{
  auto & sub = it->second;
  if (sub.requesters.empty()) {
    RCLCPP_INFO(
      kSubscriptionRegistryLogger,
      "event=subscription_pruned resource=%s kind=%s reason=%s",
      sub.resource.c_str(),
      subscriptionKindToString(sub.target_kind),
      requesterRemovalReasonToString(reason));
    destroyResource(sub);
    return subscriptions_.erase(it);
  }

  if (auto * data = sub.data_track_ptr()) {
    data->applied_interval_ms = computeAppliedIntervalMs(sub.requesters);
  }
  ++it;
  return it;
}

void SubscriptionRegistry::handleSerializedMessage(const std::string & topic, const rclcpp::SerializedMessage & message)
{
  auto it = subscriptions_.find(makeSubscriptionKey(SubscriptionTargetKind::Topic, topic));
  if (it == subscriptions_.end()) {
    return;
  }

  auto & sub = it->second;
  if (sub.requesters.empty()) {
    return;
  }

  auto * data = sub.data_track_ptr();
  if (data == nullptr) {
    return;
  }

  if (shouldSkipDueToInterval(*data)) {
    return;
  }

  if (data->cdr_track_state == CdrTrackState::kPending || data->cdr_track_state == CdrTrackState::kPublished) {
    const auto & rcl_msg = message.get_rcl_serialized_message();
    try {
      send_cdr_fn_(data->track_name, rcl_msg.buffer, rcl_msg.buffer_length);
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        kSubscriptionRegistryLogger,
        *node_.get_clock(),
        5000,
        "Failed pushing CDR frame for %s: %s",
        topic.c_str(),
        exc.what());
    }
  }
}

bool SubscriptionRegistry::shouldSkipDueToInterval(DataTrackResource & resource)
{
  if (resource.applied_interval_ms == 0) {
    return false;
  }

  const auto now = Clock::now();
  if (
    resource.last_sent_time && now - *resource.last_sent_time < std::chrono::milliseconds(resource.applied_interval_ms))
  {
    return true;
  }

  resource.last_sent_time = now;
  return false;
}

std::size_t SubscriptionRegistry::registryGeneration() const
{
  return registry_generation_.load();
}

void SubscriptionRegistry::destroyResource(SubscriptionState & sub)
{
  RCLCPP_INFO(
    kSubscriptionRegistryLogger,
    "event=subscription_destroyed resource=%s kind=%s type=%s",
    sub.resource.c_str(),
    subscriptionKindToString(sub.target_kind),
    sub.interface_type.c_str());
  if (auto * data = sub.data_track_ptr()) {
    if (data->cdr_track_state == CdrTrackState::kPublished) {
      unpublish_cdr_track_fn_(data->track_name);
    }
    data->cdr_track_state = CdrTrackState::kNone;
    data->subscription_handle.reset();
    sub.data_track_resource.reset();
    // Track names are deterministic per topic, so destroying a data subscription must advance the
    // generation before any delayed publish/disconnect callback can target the replacement entry.
    registry_generation_.fetch_add(1);
  } else {
    videoSidecarSupervisor().stopSidecar(videoSidecarLaunchSpec(sub).sidecar_key);
  }
}

void SubscriptionRegistry::clearSubscriptions()
{
  auto subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  requesters_needing_cdr_replay_.clear();

  for (auto & entry : subscriptions) {
    destroyResource(entry.second);
  }
  // Ensure generation advances even if no data subscriptions existed.
  registry_generation_.fetch_add(1);
}

}  // namespace livekit_ros2_bridge
