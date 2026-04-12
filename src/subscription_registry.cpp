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
#include <optional>
#include <utility>

#include "rclcpp/logging.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kSubscriptionRegistryLogger = rclcpp::get_logger("subscription_registry");

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
  const VideoStreamConfig & video_stream_config, const SubscriptionTarget & target, const std::string & interface_type)
{
  try {
    if (target.kind == SubscriptionTargetKind::ConfiguredSource) {
      return resolveConfiguredSourceVideoStreamSpec(video_stream_config, target.name).stream_key;
    }
    return resolveRosVideoStreamSpec(video_stream_config, target.name, interface_type).stream_key;
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

SubscriptionDemand normalizeSubscriptionDemand(const SubscriptionDemand & demand)
{
  const auto & target = demand.target;
  const std::string canonical_name = target.kind == SubscriptionTargetKind::Topic
                                       ? normalizeRosResourceName(target.name)
                                       : trimConfiguredSourceName(target.name);
  if (canonical_name.empty()) {
    throw std::invalid_argument(
      target.kind == SubscriptionTargetKind::Topic
        ? "heartbeat subscription target name must normalize to a non-empty topic name"
        : "heartbeat subscription target name must trim to a non-empty configured_source name");
  }

  return SubscriptionDemand{{target.kind, canonical_name}, demand.preferred_interval_ms};
}

std::string ensureVideoStreamRunning(VideoStreamRegistry & video_stream_registry, const VideoStreamSpec & spec)
{
  try {
    return video_stream_registry.ensureStreamRunning(spec);
  } catch (const std::exception & exc) {
    throw StreamUnavailableError(exc.what());
  }
}

}  // namespace

SubscriptionRegistry::SubscriptionRegistry(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  VideoStreamRegistry * video_stream_registry,
  const VideoStreamConfig * video_stream_config,
  const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, room_connection_(room_connection)
, video_stream_registry_(video_stream_registry)
, default_video_stream_config_(makeDefaultVideoStreamConfig())
, video_stream_config_(video_stream_config == nullptr ? &default_video_stream_config_ : video_stream_config)
, subscription_qos_config_(subscription_qos_config)
{}

SubscriptionStatus SubscriptionRegistry::renewSubscription(
  const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry)
{
  requireRequesterIdentity(requester_identity);
  const SubscriptionDemand normalized_demand = normalizeSubscriptionDemand(demand);
  const auto & target = normalized_demand.target;
  if (is_shutdown_.load()) {
    throw StreamUnavailableError("Subscription registry is shut down.");
  }

  const int preferred_interval_ms = sanitizePreferredIntervalMs(normalized_demand.preferred_interval_ms.value_or(0));
  const std::string subscription_key = makeSubscriptionKey(target.kind, target.name);
  const RequesterLease requester_lease{preferred_interval_ms, expiry};

  auto it = subscriptions_.find(subscription_key);
  if (it != subscriptions_.end()) {
    try {
      renewExistingLease(it->second, requester_identity, requester_lease);
    } catch (const std::exception & exc) {
      const auto & sub = it->second;
      LogEvent event(kSubscriptionRegistryLogger, "subscription_renew_failed");
      event.field("resource", sub.resource)
        .field("kind", subscriptionTargetKindString(sub.target_kind))
        .field("requester_identity", requester_identity);
      if (const auto * video = std::get_if<SubscriptionRegistry::VideoTrackResource>(&sub.resource_state)) {
        event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
      } else if (const auto * data = dataStreamInstance(sub)) {
        event.field("track_name", data->trackName());
      }
      event.field("error", exc.what()).warn();
      throw;
    }
    return makeSubscriptionStatus(it->second);
  }

  SubscriptionState sub;
  std::string interface_type;
  try {
    if (target.kind == SubscriptionTargetKind::ConfiguredSource) {
      sub = createVideoSubscription(normalized_demand, "", requester_identity, requester_lease);
    } else {
      interface_type = requireSingleInterfaceType(node_.get_topic_names_and_types(), target.name, "topic");
      if (classifyRosVideoInterfaceType(interface_type).has_value()) {
        sub = createVideoSubscription(normalized_demand, interface_type, requester_identity, requester_lease);
      } else {
        sub = createDataSubscription(normalized_demand, interface_type, requester_identity, requester_lease);
      }
    }
  } catch (const std::exception & exc) {
    const bool is_video_target = target.kind == SubscriptionTargetKind::ConfiguredSource ||
                                 (!interface_type.empty() && classifyRosVideoInterfaceType(interface_type).has_value());
    const std::optional<std::string> stream_key =
      is_video_target ? tryResolveVideoStreamKey(*video_stream_config_, target, interface_type) : std::nullopt;
    LogEvent event(kSubscriptionRegistryLogger, "subscription_renew_failed");
    event.field("resource", target.name)
      .field("kind", subscriptionTargetKindString(target.kind))
      .field("requester_identity", requester_identity);
    if (stream_key.has_value()) {
      event.field("stream_key", *stream_key);
    }
    event.field("error", exc.what()).warn();
    throw;
  }

  const std::string & resource = sub.resource;
  const SubscriptionTargetKind target_kind = sub.target_kind;
  subscriptions_.emplace(subscription_key, std::move(sub));
  auto inserted_it = subscriptions_.find(subscription_key);

  if (auto * data = dataStreamInstance(inserted_it->second)) {
    data->start(requester_identity);
  }

  SubscriptionStatus subscription_status = makeSubscriptionStatus(inserted_it->second);
  LogEvent event(kSubscriptionRegistryLogger, "subscription_created");
  event.field("resource", resource)
    .field("kind", subscriptionTargetKindString(target_kind))
    .field("delivery", subscriptionDeliveryKindString(subscription_status.delivery_kind))
    .field("requester_identity", requester_identity);
  if (const auto * video = std::get_if<VideoTrackResource>(&inserted_it->second.resource_state)) {
    event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
  } else if (const auto * data = dataStreamInstance(inserted_it->second)) {
    event.field("track_name", data->trackName());
  }
  event.info();
  return subscription_status;
}

SubscriptionStatus SubscriptionRegistry::renewSubscription(
  const std::string & requester_identity,
  const std::string & topic,
  int preferred_interval_ms,
  Clock::time_point expiry)
{
  return renewSubscription(
    requester_identity, SubscriptionDemand{{SubscriptionTargetKind::Topic, topic}, preferred_interval_ms}, expiry);
}

void SubscriptionRegistry::markRequesterForDataTrackRepublish(
  const std::string & requester_identity, std::size_t generation)
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
    const auto * data = dataStreamInstance(sub);
    if (data == nullptr || data->state() != DataStreamInstance::State::kPublished) {
      continue;
    }
    if (sub.requesters.find(requester_identity) == sub.requesters.end()) {
      continue;
    }

    requesters_needing_data_track_republish_.insert(requester_identity);
    return;
  }
}

void SubscriptionRegistry::republishDataTracksForRequester(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);
  if (requesters_needing_data_track_republish_.erase(requester_identity) == 0U) {
    return;
  }

  for (auto & [subscription_key, sub] : subscriptions_) {
    (void)subscription_key;
    auto * data = dataStreamInstance(sub);
    if (data == nullptr || data->state() != DataStreamInstance::State::kPublished) {
      continue;
    }
    if (sub.requesters.find(requester_identity) == sub.requesters.end()) {
      continue;
    }

    LogEvent(kSubscriptionRegistryLogger, "data_track_republish")
      .field("resource", sub.resource)
      .field("kind", "topic")
      .field("track_name", data->trackName())
      .field("requester_identity", requester_identity)
      .info();
    data->republish(requester_identity);
  }
}

void SubscriptionRegistry::renewExistingLease(
  SubscriptionState & sub, const std::string & requester_identity, const RequesterLease & requester_lease)
{
  const bool requester_already_present = sub.requesters.find(requester_identity) != sub.requesters.end();
  auto updated_requesters = sub.requesters;
  updated_requesters[requester_identity] = requester_lease;

  if (auto * data = dataStreamInstance(sub)) {
    sub.requesters = std::move(updated_requesters);
    data->updateAppliedIntervalMs(computeAppliedIntervalMs(sub.requesters));
    if (!requester_already_present && data->state() == DataStreamInstance::State::kPublished) {
      // A new requester can receive subscription status before LiveKit surfaces the existing published
      // data track to that participant session, so queue one republish when the requester first
      // joins.
      requesters_needing_data_track_republish_.insert(requester_identity);
    }
    if (data->state() == DataStreamInstance::State::kNone || data->state() == DataStreamInstance::State::kFailed) {
      data->start(requester_identity);
    }
    return;
  }

  const std::string track_name = ensureVideoStreamRunning(videoStreamRegistry(), videoStreamSpec(sub));
  sub.requesters = std::move(updated_requesters);
  std::get<VideoTrackResource>(sub.resource_state).track_name = track_name;
}

SubscriptionRegistry::SubscriptionState SubscriptionRegistry::createVideoSubscription(
  const SubscriptionDemand & demand,
  const std::string & interface_type,
  const std::string & requester_identity,
  const RequesterLease & requester_lease)
{
  const auto & target = demand.target;
  SubscriptionState sub;
  sub.target_kind = target.kind;
  sub.resource = target.name;
  sub.interface_type = interface_type;
  sub.requesters.emplace(requester_identity, requester_lease);

  const VideoStreamSpec stream_spec = target.kind == SubscriptionTargetKind::ConfiguredSource
                                        ? resolveConfiguredSourceVideoStreamSpec(*video_stream_config_, target.name)
                                        : resolveRosVideoStreamSpec(*video_stream_config_, target.name, interface_type);
  assignVideoMetadata(sub, stream_spec, ensureVideoStreamRunning(videoStreamRegistry(), stream_spec));
  return sub;
}

SubscriptionRegistry::SubscriptionState SubscriptionRegistry::createDataSubscription(
  const SubscriptionDemand & demand,
  const std::string & interface_type,
  const std::string & requester_identity,
  const RequesterLease & requester_lease)
{
  const auto & target = demand.target;
  SubscriptionState sub;
  sub.target_kind = target.kind;
  sub.resource = target.name;
  sub.interface_type = interface_type;
  sub.requesters.emplace(requester_identity, requester_lease);
  sub.resource_state = createDataStreamInstance(target.name, interface_type, sub.requesters);
  return sub;
}

void SubscriptionRegistry::assignVideoMetadata(
  SubscriptionState & sub, VideoStreamSpec stream_spec, std::string track_name)
{
  sub.resource_state = VideoTrackResource{std::move(track_name), std::move(stream_spec)};
}

std::shared_ptr<DataStreamInstance> SubscriptionRegistry::createDataStreamInstance(
  const std::string & topic,
  const std::string & interface_type,
  const std::map<std::string, RequesterLease> & requesters)
{
  return DataStreamInstance::create(
    node_,
    room_connection_,
    *this,
    topic,
    interface_type,
    computeAppliedIntervalMs(requesters),
    registry_generation_.load(),
    message_callback_gate_,
    subscription_qos_config_);
}

VideoStreamRegistry & SubscriptionRegistry::videoStreamRegistry() const
{
  if (video_stream_registry_ == nullptr) {
    throw StreamUnavailableError("Video stream registry is unavailable.");
  }

  return *video_stream_registry_;
}

const VideoStreamSpec & SubscriptionRegistry::videoStreamSpec(const SubscriptionState & sub) const
{
  const auto * video = std::get_if<VideoTrackResource>(&sub.resource_state);
  if (video == nullptr) {
    throw std::logic_error("video subscription invariant violated: video stream spec is required");
  }

  return video->stream_spec;
}

DataStreamInstance * SubscriptionRegistry::dataStreamInstance(SubscriptionState & sub)
{
  auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&sub.resource_state);
  return data == nullptr ? nullptr : data->get();
}

const DataStreamInstance * SubscriptionRegistry::dataStreamInstance(const SubscriptionState & sub)
{
  const auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&sub.resource_state);
  return data == nullptr ? nullptr : data->get();
}

void SubscriptionRegistry::revokeRequesterLeases(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);

  revokeRequesterLeasesIf(
    [&requester_identity](const std::string & candidate_requester_identity, const RequesterLease &) {
      return candidate_requester_identity == requester_identity;
    },
    RequesterLeaseRemovalReason::kParticipantDisconnected,
    Clock::now());
}

void SubscriptionRegistry::pruneExpiredLeases()
{
  if (is_shutdown_.load()) {
    return;
  }
  const auto now = Clock::now();
  revokeRequesterLeasesIf(
    [now](const std::string &, const RequesterLease & requester_lease) { return now >= requester_lease.expiry; },
    RequesterLeaseRemovalReason::kLeaseExpired,
    now);
}

bool SubscriptionRegistry::hasSubscription(const std::string & resource, SubscriptionTargetKind target_kind) const
{
  const std::string canonical_name = target_kind == SubscriptionTargetKind::Topic ? normalizeRosResourceName(resource)
                                                                                  : trimConfiguredSourceName(resource);
  if (canonical_name.empty()) {
    return false;
  }
  return subscriptions_.find(makeSubscriptionKey(target_kind, canonical_name)) != subscriptions_.end();
}

void SubscriptionRegistry::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }
  LogEvent(kSubscriptionRegistryLogger, "subscription_registry_reset_begin")
    .field("resource", "subscriptions")
    .field("subscription_count", subscriptions_.size())
    .field("pending_data_track_republishes", requesters_needing_data_track_republish_.size())
    .info();
  const std::size_t callback_generation = message_callback_gate_.close();
  requesters_needing_data_track_republish_.clear();
  clearSubscriptions();
  message_callback_gate_.open(callback_generation);
}

void SubscriptionRegistry::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  LogEvent(kSubscriptionRegistryLogger, "subscription_registry_shutdown_begin")
    .field("resource", "subscriptions")
    .field("subscription_count", subscriptions_.size())
    .field("pending_data_track_republishes", requesters_needing_data_track_republish_.size())
    .info();
  (void)message_callback_gate_.close();
  clearSubscriptions();
}

bool SubscriptionRegistry::onDataTrackPublished(const std::string & track_name, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return false;
  }
  auto it = findByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return false;
  }
  auto * data = dataStreamInstance(it->second);
  if (data == nullptr) {
    return false;
  }
  return data->onPublishComplete(generation);
}

void SubscriptionRegistry::onDataTrackFailed(const std::string & track_name)
{
  if (is_shutdown_.load()) {
    return;
  }
  auto it = findByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return;
  }
  auto * data = dataStreamInstance(it->second);
  if (data == nullptr) {
    return;
  }
  data->onPublishFailed();
}

SubscriptionStatus SubscriptionRegistry::makeSubscriptionStatus(const SubscriptionState & sub)
{
  SubscriptionStatus subscription_status;
  subscription_status.target = {sub.target_kind, sub.resource};
  subscription_status.interface_type = sub.interface_type;
  if (const auto * data = dataStreamInstance(sub)) {
    subscription_status.delivery_kind = SubscriptionDeliveryKind::kData;
    if (data->state() == DataStreamInstance::State::kPending || data->state() == DataStreamInstance::State::kPublished)
    {
      subscription_status.track_name = data->trackName();
    }
    subscription_status.applied_interval_ms = data->appliedIntervalMs();
  } else {
    const auto & video = std::get<VideoTrackResource>(sub.resource_state);
    subscription_status.delivery_kind = SubscriptionDeliveryKind::kVideo;
    subscription_status.track_name = video.track_name;
    subscription_status.degraded_reason = video.stream_spec.degraded_reason.value_or("");
  }

  return subscription_status;
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

std::string SubscriptionRegistry::makeSubscriptionKey(SubscriptionTargetKind target_kind, const std::string & resource)
{
  return std::string(subscriptionTargetKindString(target_kind)) + ":" + resource;
}

void SubscriptionRegistry::revokeRequesterLeasesIf(
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
      event.field("resource", sub.resource)
        .field("kind", subscriptionTargetKindString(sub.target_kind))
        .field("requester_identity", requester_identity)
        .field("reason", requesterRemovalReasonToString(reason))
        .field("preferred_interval_ms", requester_lease.preferred_interval_ms)
        .field("remaining_requesters", remaining_requesters)
        .field(time_label, time_value)
        .info();

      removed_any = true;
      requesters_needing_data_track_republish_.erase(requester_identity);
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
    auto * data = dataStreamInstance(sub);
    if (data != nullptr && data->trackName() == track_name) {
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
    if (const auto * data = dataStreamInstance(sub)) {
      LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
        .field("resource", sub.resource)
        .field("kind", subscriptionTargetKindString(sub.target_kind))
        .field("reason", requesterRemovalReasonToString(reason))
        .field("track_name", data->trackName())
        .info();
    } else {
      const auto & video = std::get<VideoTrackResource>(sub.resource_state);
      LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
        .field("resource", sub.resource)
        .field("kind", subscriptionTargetKindString(sub.target_kind))
        .field("reason", requesterRemovalReasonToString(reason))
        .field("stream_key", video.stream_spec.stream_key)
        .field("track_name", video.track_name)
        .info();
    }
    destroyResource(sub);
    return subscriptions_.erase(it);
  }

  if (auto * data = dataStreamInstance(sub)) {
    data->updateAppliedIntervalMs(computeAppliedIntervalMs(sub.requesters));
  }
  ++it;
  return it;
}

std::size_t SubscriptionRegistry::registryGeneration() const
{
  return registry_generation_.load();
}

void SubscriptionRegistry::destroyResource(SubscriptionState & sub)
{
  if (auto * data = dataStreamInstance(sub)) {
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .field("resource", sub.resource)
      .field("kind", subscriptionTargetKindString(sub.target_kind))
      .field("interface_type", sub.interface_type)
      .field("track_name", data->trackName())
      .info();
    data->shutdown();
    sub.resource_state = std::shared_ptr<DataStreamInstance>{};
    // Track names are deterministic per topic, so destroying a data subscription must advance the
    // generation before any delayed publish/disconnect callback can target the replacement entry.
    registry_generation_.fetch_add(1);
  } else {
    const auto & video = std::get<VideoTrackResource>(sub.resource_state);
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .field("resource", sub.resource)
      .field("kind", subscriptionTargetKindString(sub.target_kind))
      .field("interface_type", sub.interface_type)
      .field("stream_key", video.stream_spec.stream_key)
      .field("track_name", video.track_name)
      .info();
    videoStreamRegistry().stopStream(video.stream_spec.stream_key);
  }
}

void SubscriptionRegistry::clearSubscriptions()
{
  auto subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  requesters_needing_data_track_republish_.clear();

  for (auto & entry : subscriptions) {
    destroyResource(entry.second);
  }
  // Ensure generation advances even if no data subscriptions existed.
  registry_generation_.fetch_add(1);
}

}  // namespace livekit_ros2_bridge
