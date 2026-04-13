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
#include "utils/trim.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kSubscriptionRegistryLogger = rclcpp::get_logger("subscription_registry");

void requireRequesterIdentity(const std::string & requester_identity)
{
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }
}

std::string startVideoStream(VideoStreamRegistry & video_stream_registry, const VideoStreamSpec & stream_spec)
{
  try {
    return video_stream_registry.start(stream_spec);
  } catch (const std::exception & exc) {
    throw StreamUnavailableError(exc.what());
  }
}

const char * leaseRemovalReasonString(LeaseRemovalReason reason)
{
  switch (reason) {
    case LeaseRemovalReason::kParticipantDisconnected:
      return "participant_disconnected";
    case LeaseRemovalReason::kLeaseExpired:
      return "lease_expired";
  }

  return "unknown";
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
  const auto & requested_target = demand.target;
  const std::string canonical_name = requested_target.kind == SubscriptionTargetKind::Topic
                                       ? normalizeRosResourceName(requested_target.name)
                                       : trim(requested_target.name);
  if (canonical_name.empty()) {
    throw std::invalid_argument(
      requested_target.kind == SubscriptionTargetKind::Topic
        ? "heartbeat subscription target name must normalize to a non-empty topic name"
        : "heartbeat subscription target name must trim to a non-empty configured_source name");
  }
  const SubscriptionTarget target{requested_target.kind, canonical_name};
  if (is_shutdown_.load()) {
    throw StreamUnavailableError("Subscription registry is shut down.");
  }

  const int preferred_interval_ms = std::max(demand.preferred_interval_ms.value_or(0), 0);
  const std::string key = keyFor(target.kind, target.name);
  const Lease lease{preferred_interval_ms, expiry};

  auto it = subscriptions_.find(key);
  if (it != subscriptions_.end()) {
    try {
      auto & entry = it->second;
      const bool had_requester = entry.leases.find(requester_identity) != entry.leases.end();
      entry.leases[requester_identity] = lease;

      if (auto * data = dataStream(entry)) {
        const std::size_t publish_generation = registry_generation_.load();
        data->setSuppressionIntervalMs(appliedIntervalMs(entry.leases));
        if (!had_requester && data->state() == DataStreamInstance::State::kPublished) {
          // A new requester can receive subscription status before LiveKit surfaces the existing
          // published data track to that participant session, so queue one republish when the
          // requester first joins. The fresh lease was inserted just above, so only published
          // state matters here.
          republish_requesters_.insert(requester_identity);
        }
        if (data->state() == DataStreamInstance::State::kNone || data->state() == DataStreamInstance::State::kFailed) {
          data->start(requester_identity, publish_generation);
        }
      } else {
        auto * video = std::get_if<VideoRuntime>(&entry.runtime);
        if (video == nullptr) {
          throw std::logic_error("video subscription invariant violated: video stream spec is required");
        }
        // VideoStreamRegistry shares one runtime per resolved stream key, so renew reuses that
        // runtime and updates the status-visible track name from the shared instance.
        video->track_name = startVideoStream(videoRegistry(), video->stream_spec);
      }
    } catch (const std::exception & exc) {
      const auto & entry = it->second;
      LogEvent event(kSubscriptionRegistryLogger, "subscription_renew_failed");
      event.field("resource", entry.name)
        .field("kind", subscriptionTargetKindString(entry.target_kind))
        .field("requester_identity", requester_identity);
      if (const auto * video = std::get_if<VideoRuntime>(&entry.runtime)) {
        event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
      } else if (const auto * data = dataStream(entry)) {
        event.field("track_name", data->trackName());
      }
      event.field("error", exc.what()).warn();
      throw;
    }
    return statusFor(it->second);
  }

  bool is_video = target.kind == SubscriptionTargetKind::ConfiguredSource;
  std::string interface_type;
  std::optional<std::string> stream_key;
  Entry entry;
  entry.target_kind = target.kind;
  entry.name = target.name;
  try {
    if (!is_video) {
      interface_type = requireSingleInterfaceType(node_.get_topic_names_and_types(), target.name, "topic");
      is_video = classifyRosVideoIngestMode(interface_type).has_value();
    }
    entry.interface_type = interface_type;
    entry.leases.emplace(requester_identity, lease);

    if (is_video) {
      VideoStreamSpec stream_spec = target.kind == SubscriptionTargetKind::ConfiguredSource
                                      ? resolveConfiguredVideoSourceSpec(*video_stream_config_, target.name)
                                      : resolveRosVideoTopicSpec(*video_stream_config_, target.name, interface_type);
      stream_key = stream_spec.stream_key;
      std::string track_name = startVideoStream(videoRegistry(), stream_spec);
      entry.runtime = VideoRuntime{std::move(track_name), std::move(stream_spec)};
    } else {
      auto data = std::shared_ptr<DataStreamInstance>(new DataStreamInstance(
        target.name, interface_type, node_, room_connection_, *this, message_callback_gate_, subscription_qos_config_));
      data->setSuppressionIntervalMs(appliedIntervalMs(entry.leases));
      data->subscribe();
      entry.runtime = std::move(data);
    }
  } catch (const std::exception & exc) {
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

  const std::string & resource = entry.name;
  const SubscriptionTargetKind target_kind = entry.target_kind;
  // Make the entry visible before starting a data-track publish: completion callbacks resolve
  // back into the registry by deterministic track name via findDataByTrackName().
  subscriptions_.emplace(key, std::move(entry));
  auto inserted = subscriptions_.find(key);

  if (auto * data = dataStream(inserted->second)) {
    const std::size_t publish_generation = registry_generation_.load();
    data->start(requester_identity, publish_generation);
  }

  SubscriptionStatus status = statusFor(inserted->second);
  LogEvent event(kSubscriptionRegistryLogger, "subscription_created");
  event.field("resource", resource)
    .field("kind", subscriptionTargetKindString(target_kind))
    .field("delivery", subscriptionDeliveryKindString(status.delivery_kind))
    .field("requester_identity", requester_identity);
  if (const auto * video = std::get_if<VideoRuntime>(&inserted->second.runtime)) {
    event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
  } else if (const auto * data = dataStream(inserted->second)) {
    event.field("track_name", data->trackName());
  }
  event.info();
  return status;
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

void SubscriptionRegistry::queueDataTrackRepublish(const std::string & requester_identity, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);
  if (generation != registry_generation_.load()) {
    return;
  }

  for (const auto & [subscription_key, entry] : subscriptions_) {
    (void)subscription_key;
    const auto * data = dataStream(entry);
    if (
      data == nullptr || data->state() != DataStreamInstance::State::kPublished ||
      entry.leases.find(requester_identity) == entry.leases.end())
    {
      continue;
    }

    // The republish queue is keyed only by requester. Once any currently published data track
    // proves this requester still owns a live lease, republishDataTracks() will sweep the rest.
    republish_requesters_.insert(requester_identity);
    return;
  }
}

void SubscriptionRegistry::republishDataTracks(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);
  if (republish_requesters_.erase(requester_identity) == 0U) {
    return;
  }

  const std::size_t publish_generation = registry_generation_.load();
  for (auto & [subscription_key, entry] : subscriptions_) {
    (void)subscription_key;
    auto * data = dataStream(entry);
    if (
      data == nullptr || data->state() != DataStreamInstance::State::kPublished ||
      entry.leases.find(requester_identity) == entry.leases.end())
    {
      continue;
    }

    LogEvent(kSubscriptionRegistryLogger, "data_track_republish")
      .field("resource", entry.name)
      .field("kind", "topic")
      .field("track_name", data->trackName())
      .field("requester_identity", requester_identity)
      .info();
    data->republish(requester_identity, publish_generation);
  }
}

VideoStreamRegistry & SubscriptionRegistry::videoRegistry() const
{
  if (video_stream_registry_ == nullptr) {
    throw StreamUnavailableError("Video stream registry is unavailable.");
  }

  return *video_stream_registry_;
}

DataStreamInstance * SubscriptionRegistry::dataStream(Entry & entry)
{
  auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&entry.runtime);
  return data == nullptr ? nullptr : data->get();
}

const DataStreamInstance * SubscriptionRegistry::dataStream(const Entry & entry)
{
  const auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&entry.runtime);
  return data == nullptr ? nullptr : data->get();
}

void SubscriptionRegistry::revokeRequesterLeases(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  requireRequesterIdentity(requester_identity);

  removeLeasesIf(
    [&requester_identity](const std::string & candidate_requester_identity, const Lease &) {
      return candidate_requester_identity == requester_identity;
    },
    LeaseRemovalReason::kParticipantDisconnected,
    Clock::now());
}

void SubscriptionRegistry::pruneExpiredLeases()
{
  if (is_shutdown_.load()) {
    return;
  }
  const auto now = Clock::now();
  removeLeasesIf(
    [now](const std::string &, const Lease & lease) { return now >= lease.expiry; },
    LeaseRemovalReason::kLeaseExpired,
    now);
}

bool SubscriptionRegistry::hasSubscription(const std::string & resource, SubscriptionTargetKind target_kind) const
{
  const std::string canonical_name =
    target_kind == SubscriptionTargetKind::Topic ? normalizeRosResourceName(resource) : trim(resource);
  if (canonical_name.empty()) {
    return false;
  }
  return subscriptions_.find(keyFor(target_kind, canonical_name)) != subscriptions_.end();
}

void SubscriptionRegistry::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }
  LogEvent(kSubscriptionRegistryLogger, "subscription_registry_reset_begin")
    .field("resource", "subscriptions")
    .field("subscription_count", subscriptions_.size())
    .field("pending_data_track_republishes", republish_requesters_.size())
    .info();

  const std::size_t callback_generation = message_callback_gate_.close();
  republish_requesters_.clear();
  clearSubscriptions();
  // Reopen only after old-session teardown completes so new callbacks cannot interleave with it.
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
    .field("pending_data_track_republishes", republish_requesters_.size())
    .info();

  (void)message_callback_gate_.close();
  clearSubscriptions();
}

bool SubscriptionRegistry::onDataTrackPublished(const std::string & track_name, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return false;
  }
  auto it = findDataByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return false;
  }
  auto * data = dataStream(it->second);
  if (data == nullptr) {
    return false;
  }
  return data->completePublish(generation);
}

void SubscriptionRegistry::onDataTrackFailed(const std::string & track_name)
{
  if (is_shutdown_.load()) {
    return;
  }
  auto it = findDataByTrackName(track_name);
  if (it == subscriptions_.end()) {
    return;
  }
  auto * data = dataStream(it->second);
  if (data == nullptr) {
    return;
  }
  data->failPublish();
}

SubscriptionStatus SubscriptionRegistry::statusFor(const Entry & entry)
{
  SubscriptionStatus status;
  status.target = {entry.target_kind, entry.name};
  status.interface_type = entry.interface_type;
  if (const auto * data = dataStream(entry)) {
    status.delivery_kind = SubscriptionDeliveryKind::kData;
    if (data->state() == DataStreamInstance::State::kPending || data->state() == DataStreamInstance::State::kPublished)
    {
      status.track_name = data->trackName();
    }
    status.applied_interval_ms = data->suppressionIntervalMs();
  } else {
    const auto & video = std::get<VideoRuntime>(entry.runtime);
    status.delivery_kind = SubscriptionDeliveryKind::kVideo;
    status.track_name = video.track_name;
    status.degraded_reason = video.stream_spec.degraded_reason.value_or("");
  }

  return status;
}

int SubscriptionRegistry::appliedIntervalMs(const std::map<std::string, Lease> & leases)
{
  if (leases.empty()) {
    return 0;
  }

  int applied_interval_ms = leases.begin()->second.preferred_interval_ms;
  for (const auto & [id, lease] : leases) {
    (void)id;
    applied_interval_ms = std::min(applied_interval_ms, lease.preferred_interval_ms);
  }
  return applied_interval_ms;
}

std::string SubscriptionRegistry::keyFor(SubscriptionTargetKind target_kind, const std::string & resource)
{
  return std::string(subscriptionTargetKindString(target_kind)) + ":" + resource;
}

void SubscriptionRegistry::removeLeasesIf(
  const LeasePredicate & should_remove, LeaseRemovalReason reason, Clock::time_point reference_time)
{
  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto & entry = it->second;
    bool removed_any = false;

    for (auto req_it = entry.leases.begin(); req_it != entry.leases.end();) {
      const auto & requester_identity = req_it->first;
      const auto & lease = req_it->second;
      if (!should_remove(requester_identity, lease)) {
        ++req_it;
        continue;
      }

      const auto remaining_requesters = entry.leases.size() - 1U;
      const auto delta_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(lease.expiry - reference_time).count();
      const char * time_label = reason == LeaseRemovalReason::kLeaseExpired ? "expired_by_ms" : "expires_in_ms";
      const long time_value =
        reason == LeaseRemovalReason::kLeaseExpired ? static_cast<long>(-delta_ms) : static_cast<long>(delta_ms);
      LogEvent event(kSubscriptionRegistryLogger, "requester_lease_removed");
      event.field("resource", entry.name)
        .field("kind", subscriptionTargetKindString(entry.target_kind))
        .field("requester_identity", requester_identity)
        .field("reason", leaseRemovalReasonString(reason))
        .field("preferred_interval_ms", lease.preferred_interval_ms)
        .field("remaining_requesters", remaining_requesters)
        .field(time_label, time_value)
        .info();

      removed_any = true;
      republish_requesters_.erase(requester_identity);
      req_it = entry.leases.erase(req_it);
    }

    if (!removed_any) {
      ++it;
      continue;
    }

    if (entry.leases.empty()) {
      if (const auto * data = dataStream(entry)) {
        LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
          .field("resource", entry.name)
          .field("kind", subscriptionTargetKindString(entry.target_kind))
          .field("reason", leaseRemovalReasonString(reason))
          .field("track_name", data->trackName())
          .info();
      } else {
        const auto & video = std::get<VideoRuntime>(entry.runtime);
        LogEvent(kSubscriptionRegistryLogger, "subscription_pruned")
          .field("resource", entry.name)
          .field("kind", subscriptionTargetKindString(entry.target_kind))
          .field("reason", leaseRemovalReasonString(reason))
          .field("stream_key", video.stream_spec.stream_key)
          .field("track_name", video.track_name)
          .info();
      }
      destroyRuntime(entry);
      it = subscriptions_.erase(it);
      continue;
    }

    if (auto * data = dataStream(entry)) {
      data->setSuppressionIntervalMs(appliedIntervalMs(entry.leases));
    }
    ++it;
  }
}

SubscriptionRegistry::EntryMap::iterator SubscriptionRegistry::findDataByTrackName(const std::string & track_name)
{
  // Registry storage is keyed by canonical subscription target, but data publish callbacks only
  // identify the deterministic LiveKit track name.
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    auto & entry = it->second;
    auto * data = dataStream(entry);
    if (data != nullptr && data->trackName() == track_name) {
      return it;
    }
  }
  return subscriptions_.end();
}

std::size_t SubscriptionRegistry::generation() const
{
  return registry_generation_.load();
}

void SubscriptionRegistry::destroyRuntime(Entry & entry)
{
  if (auto * data = dataStream(entry)) {
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .field("resource", entry.name)
      .field("kind", subscriptionTargetKindString(entry.target_kind))
      .field("interface_type", entry.interface_type)
      .field("track_name", data->trackName())
      .info();
    data->shutdown();
    entry.runtime = std::shared_ptr<DataStreamInstance>{};
    // Track names are deterministic per topic, so destroying a data subscription must advance the
    // generation before any delayed publish/disconnect callback can target the replacement entry.
    registry_generation_.fetch_add(1);
  } else {
    const auto & video = std::get<VideoRuntime>(entry.runtime);
    LogEvent(kSubscriptionRegistryLogger, "subscription_destroyed")
      .field("resource", entry.name)
      .field("kind", subscriptionTargetKindString(entry.target_kind))
      .field("interface_type", entry.interface_type)
      .field("stream_key", video.stream_spec.stream_key)
      .field("track_name", video.track_name)
      .info();
    videoRegistry().stop(video.stream_spec.stream_key);
  }
}

void SubscriptionRegistry::clearSubscriptions()
{
  // Remove visible registry state first so late publish/disconnect callbacks cannot resolve a
  // soon-to-be-destroyed entry while teardown is running.
  auto entries = std::move(subscriptions_);
  subscriptions_.clear();
  republish_requesters_.clear();

  for (auto & entry : entries) {
    destroyRuntime(entry.second);
  }
  // Ensure generation advances even if no data subscriptions existed.
  registry_generation_.fetch_add(1);
}

}  // namespace livekit_ros2_bridge
