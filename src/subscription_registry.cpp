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
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/scope_exit.hpp"
#include "video_stream_manager.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr std::size_t kDataSubscriptionDepth = 10U;
constexpr auto kTrackDeliveryFailureLogThrottlePeriod = std::chrono::seconds(5);
constexpr char kTopicSubscriptionKeyPrefix[] = "topic:";
constexpr char kExternalSubscriptionKeyPrefix[] = "external:";
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

std::optional<std::string> tryResolveVideoStreamKey(
  const VideoConfig & video_config, const SubscriptionTarget & target, const std::string & interface_type)
{
  try {
    if (target.kind == SubscriptionTargetKind::External) {
      return resolvePipelineVideoLaunchSpec(video_config, target.name).stream_key;
    }
    return resolveRosVideoLaunchSpec(video_config, target.name, interface_type).stream_key;
  } catch (...) {
    return std::nullopt;
  }
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

std::string ensureVideoStream(VideoStreamManager & video_stream_manager, const SidecarLaunchSpec & spec)
{
  try {
    return video_stream_manager.ensureStream(spec);
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
  VideoStreamManager * video_stream_manager,
  const VideoConfig * video_config)
: node_(node)
, send_cdr_fn_(std::move(send_cdr_fn))
, publish_cdr_track_fn_(std::move(publish_cdr_track_fn))
, unpublish_cdr_track_fn_(std::move(unpublish_cdr_track_fn))
, video_stream_manager_(video_stream_manager)
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
    try {
      refreshExistingLease(it->second, requester_identity, requester_lease);
    } catch (const std::exception & exc) {
      const auto & sub = it->second;
      LogEvent event(kSubscriptionRegistryLogger, "subscription_renew_failed");
      event.kv("resource", sub.resource)
        .kv("kind", subscriptionKindToString(sub.target_kind))
        .kv("requester_identity", requester_identity);
      if (sub.video_stream_spec.has_value()) {
        event.kv("stream_key", sub.video_stream_spec->stream_key).kv("track_name", sub.video_track_name);
      }
      event.kv("error", exc.what()).warn();
      throw;
    }
    return makeStreamStatus(it->second);
  }

  SubscriptionState sub;
  std::string interface_type;
  try {
    if (target.kind == SubscriptionTargetKind::External) {
      sub = createVideoSubscription(normalized, "", requester_identity, requester_lease);
    } else {
      interface_type = requireUniqueInterfaceType(node_.get_topic_names_and_types(), target.name, "topic");
      if (classifyRosVideoInterfaceType(interface_type).has_value()) {
        sub = createVideoSubscription(normalized, interface_type, requester_identity, requester_lease);
      } else {
        sub = createDataSubscription(normalized, interface_type, requester_identity, requester_lease);
      }
    }
  } catch (const std::exception & exc) {
    const bool is_video_target = target.kind == SubscriptionTargetKind::External ||
                                 (!interface_type.empty() && classifyRosVideoInterfaceType(interface_type).has_value());
    const std::optional<std::string> stream_key =
      is_video_target ? tryResolveVideoStreamKey(*video_config_, target, interface_type) : std::nullopt;
    LogEvent event(kSubscriptionRegistryLogger, "subscription_renew_failed");
    event.kv("resource", target.name)
      .kv("kind", subscriptionKindToString(target.kind))
      .kv("requester_identity", requester_identity);
    if (stream_key.has_value()) {
      event.kv("stream_key", *stream_key);
    }
    event.kv("error", exc.what()).warn();
    throw;
  }

  StreamStatus stream_status = makeStreamStatus(sub);
  LogEvent event(kSubscriptionRegistryLogger, "subscription_created");
  event.kv("resource", sub.resource)
    .kv("kind", subscriptionKindToString(sub.target_kind))
    .kv("delivery", streamDeliveryKindString(stream_status.delivery_kind))
    .kv("requester_identity", requester_identity);
  if (sub.video_stream_spec.has_value()) {
    event.kv("stream_key", sub.video_stream_spec->stream_key).kv("track_name", sub.video_track_name);
  } else {
    const auto * data = sub.data_track_ptr();
    event.kv("track_name", data == nullptr ? "" : data->track_name);
  }
  event.info();
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

    LogEvent(kSubscriptionRegistryLogger, "cdr_track_replay")
      .kv("resource", sub.resource)
      .kv("kind", "topic")
      .kv("track_name", data->track_name)
      .kv("requester_identity", requester_identity)
      .info();
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

  const std::string track_name = ensureVideoStream(videoStreamManager(), videoStreamSpec(sub));
  sub.requesters = std::move(updated_requesters);
  sub.video_track_name = track_name;
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

  const SidecarLaunchSpec video_stream_spec =
    target.kind == SubscriptionTargetKind::External
      ? resolvePipelineVideoLaunchSpec(*video_config_, target.name)
      : resolveRosVideoLaunchSpec(*video_config_, target.name, interface_type);
  assignVideoMetadata(sub, video_stream_spec, ensureVideoStream(videoStreamManager(), video_stream_spec));
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
  SubscriptionState & sub, const SidecarLaunchSpec & video_stream_spec, std::string track_name)
{
  sub.source_kind = videoSourceKindToString(video_stream_spec.source_kind);
  sub.ingest_mode = video_stream_spec.ingest_mode;
  sub.selected_config_key = video_stream_spec.selected_config_key;
  sub.video_track_name = std::move(track_name);
  sub.video_stream_spec = video_stream_spec;
}

SubscriptionRegistry::DataTrackResource SubscriptionRegistry::createPendingDataTrackResource(
  const std::string & topic,
  const std::string & interface_type,
  const std::map<std::string, RequesterLease> & requesters,
  const std::string & requester_identity)
{
  const rclcpp::QoS qos(kDataSubscriptionDepth);

  DataTrackResource data;
  const std::size_t callback_generation = message_callback_guard_.currentGeneration();
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
      if (!message_callback_guard_.tryBeginWork(callback_generation)) {
        return;
      }
      ScopeExit finish_delivery([this]() { message_callback_guard_.endWork(); });
      handleSerializedMessage(normalized_topic, *message);
    });
  return data;
}

void SubscriptionRegistry::publishPendingCdrTrack(
  const std::string & topic, DataTrackResource & data, const std::string & requester_identity)
{
  data.cdr_track_state = CdrTrackState::kPending;
  LogEvent(kSubscriptionRegistryLogger, "cdr_track_pending")
    .kv("resource", topic)
    .kv("kind", "topic")
    .kv("track_name", data.track_name)
    .kv("requester_identity", requester_identity)
    .info();
  publish_cdr_track_fn_(data.track_name, data.generation);
}

VideoStreamManager & SubscriptionRegistry::videoStreamManager() const
{
  if (video_stream_manager_ == nullptr) {
    throw StreamUnavailableError("Video stream manager is unavailable.");
  }

  return *video_stream_manager_;
}

const SidecarLaunchSpec & SubscriptionRegistry::videoStreamSpec(const SubscriptionState & sub) const
{
  if (!sub.video_stream_spec.has_value()) {
    throw std::logic_error("video subscription invariant violated: video stream spec is required");
  }

  return *sub.video_stream_spec;
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
  LogEvent(kSubscriptionRegistryLogger, "subscription_registry_reset_begin")
    .kv("resource", "subscriptions")
    .kv("subscription_count", subscriptions_.size())
    .kv("pending_cdr_replays", requesters_needing_cdr_replay_.size())
    .info();
  const std::size_t callback_generation = message_callback_guard_.quiesce();
  requesters_needing_cdr_replay_.clear();
  clearSubscriptions();
  message_callback_guard_.resume(callback_generation);
}

void SubscriptionRegistry::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  LogEvent(kSubscriptionRegistryLogger, "subscription_registry_shutdown_begin")
    .kv("resource", "subscriptions")
    .kv("subscription_count", subscriptions_.size())
    .kv("pending_cdr_replays", requesters_needing_cdr_replay_.size())
    .info();
  (void)message_callback_guard_.quiesce();
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
  LogEvent(kSubscriptionRegistryLogger, "cdr_track_published")
    .kv("resource", sub.resource)
    .kv("kind", "topic")
    .kv("track_name", data->track_name)
    .info();
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
  LogEvent(kSubscriptionRegistryLogger, "cdr_track_publish_failed")
    .kv("resource", sub.resource)
    .kv("kind", "topic")
    .kv("track_name", data->track_name)
    .warn();
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
    stream_status.track_name = sub.video_track_name;
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
  return (target_kind == SubscriptionTargetKind::Topic ? kTopicSubscriptionKeyPrefix : kExternalSubscriptionKeyPrefix) +
         resource;
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
      LogEvent event(kSubscriptionRegistryLogger, "requester_lease_removed");
      event.kv("resource", sub.resource)
        .kv("kind", subscriptionKindToString(sub.target_kind))
        .kv("requester_identity", requester_identity)
        .kv("reason", requesterRemovalReasonToString(reason))
        .kv("preferred_interval_ms", requester_lease.preferred_interval_ms)
        .kv("remaining_requesters", remaining_requesters)
        .kv(time_label, time_value)
        .info();

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
    if (const auto * data = sub.data_track_ptr()) {
      LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
        .kv("resource", sub.resource)
        .kv("kind", subscriptionKindToString(sub.target_kind))
        .kv("reason", requesterRemovalReasonToString(reason))
        .kv("track_name", data->track_name)
        .info();
    } else {
      LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
        .kv("resource", sub.resource)
        .kv("kind", subscriptionKindToString(sub.target_kind))
        .kv("reason", requesterRemovalReasonToString(reason))
        .kv("stream_key", videoStreamSpec(sub).stream_key)
        .kv("track_name", sub.video_track_name)
        .info();
    }
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

  if (data->cdr_track_state != CdrTrackState::kPending && data->cdr_track_state != CdrTrackState::kPublished) {
    return;
  }

  const auto & rcl_msg = message.get_rcl_serialized_message();
  try {
    send_cdr_fn_(data->track_name, rcl_msg.buffer, rcl_msg.buffer_length);
  } catch (const std::exception & exc) {
    LogEvent(kSubscriptionRegistryLogger, "cdr_track_delivery_failed")
      .kv("resource", sub.resource)
      .kv("kind", "topic")
      .kv("track_name", data->track_name)
      .kv("error", exc.what())
      .warnThrottle(*node_.get_clock(), kTrackDeliveryFailureLogThrottlePeriod);
  }
}

bool SubscriptionRegistry::shouldSkipDueToInterval(DataTrackResource & resource)
{
  if (resource.applied_interval_ms == 0) {
    return false;
  }

  const auto now = Clock::now();
  const auto suppression_window = std::chrono::milliseconds(resource.applied_interval_ms);
  const bool within_suppression_window = resource.last_sent_time && now - *resource.last_sent_time < suppression_window;
  if (within_suppression_window) {
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
  if (auto * data = sub.data_track_ptr()) {
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .kv("resource", sub.resource)
      .kv("kind", subscriptionKindToString(sub.target_kind))
      .kv("interface_type", sub.interface_type)
      .kv("track_name", data->track_name)
      .info();
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
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .kv("resource", sub.resource)
      .kv("kind", subscriptionKindToString(sub.target_kind))
      .kv("interface_type", sub.interface_type)
      .kv("stream_key", videoStreamSpec(sub).stream_key)
      .kv("track_name", sub.video_track_name)
      .info();
    videoStreamManager().stopStream(videoStreamSpec(sub).stream_key);
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
