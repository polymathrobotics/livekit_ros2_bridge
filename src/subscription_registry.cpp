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

#include "rclcpp/logging.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "video_stream_registry.hpp"
#include "wire/subscriptions.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("subscription_registry");

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
  if (is_shutdown_.load()) {
    throw StreamUnavailableError("Subscription registry is shut down.");
  }
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }
  if (demand.name.empty()) {
    throw std::invalid_argument("heartbeat subscription target name must resolve to a non-empty name");
  }

  const int preferred_interval_ms = std::max(demand.preferred_interval_ms.value_or(0), 0);
  const Lease lease{preferred_interval_ms, expiry};

  if (auto * existing_subscription = findSubscription(demand.kind, demand.name)) {
    return renewExistingSubscription(*existing_subscription, requester_identity, lease);
  }

  return createSubscription(demand, requester_identity, lease);
}

SubscriptionStatus SubscriptionRegistry::renewExistingSubscription(
  SharedSubscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    const bool had_requester = subscription.leases.find(requester_identity) != subscription.leases.end();
    subscription.leases[requester_identity] = lease;

    if (auto * data = dataInstance(subscription)) {
      const std::size_t publish_generation = registry_generation_.load();
      data->setIntervalMs(appliedIntervalMs(subscription.leases));
      if (!had_requester && data->state() == DataStreamInstance::State::kPublished) {
        // A new requester can receive subscription status before LiveKit surfaces the existing
        // published data track to that participant session, so queue one republish when the
        // requester first joins. The fresh lease was inserted just above, so only published
        // state matters here.
        if (republish_requesters_.insert(requester_identity).second) {
          LogEvent(kLogger, "data_track_republish_queued")
            .field("resource", subscription.name)
            .field("track_name", data->trackName())
            .field("requester_identity", requester_identity)
            .field("reason", "new_requester")
            .info();
        }
      }
      if (data->state() == DataStreamInstance::State::kNone || data->state() == DataStreamInstance::State::kFailed) {
        data->start(publish_generation);
      }
    } else {
      auto * video = std::get_if<VideoRuntime>(&subscription.runtime);
      if (video == nullptr) {
        throw std::logic_error("video subscription invariant violated: video stream spec is required");
      }
      // VideoStreamRegistry shares one runtime per resolved stream key, so renew reuses that
      // runtime and updates the status-visible track name from the shared instance.
      try {
        video->track_name = videoRegistry().start(video->stream_spec);
      } catch (const std::exception & exc) {
        throw StreamUnavailableError(exc.what());
      }
    }
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "subscription_renew_failed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("requester_identity", requester_identity);
    if (const auto * video = std::get_if<VideoRuntime>(&subscription.runtime)) {
      event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
    } else if (const auto * data = dataInstance(subscription)) {
      event.field("track_name", data->trackName());
    }
    event.field("error", exc.what()).warn();
    throw;
  }

  return statusFor(subscription);
}

SubscriptionStatus SubscriptionRegistry::createSubscription(
  const SubscriptionDemand & demand, const std::string & requester_identity, const Lease & lease)
{
  bool is_video = demand.kind == SubscriptionTargetKind::ConfiguredSource;
  std::string interface_type;
  std::optional<std::string> stream_key;
  SharedSubscription subscription;
  subscription.target_kind = demand.kind;
  subscription.name = demand.name;
  try {
    if (!is_video) {
      interface_type = requireSingleInterfaceType(node_.get_topic_names_and_types(), demand.name, "topic");
      is_video = classifyRosVideoIngestMode(interface_type).has_value();
    }
    subscription.interface_type = interface_type;
    subscription.leases.emplace(requester_identity, lease);

    if (is_video) {
      VideoStreamSpec stream_spec = demand.kind == SubscriptionTargetKind::ConfiguredSource
                                      ? resolveConfiguredVideoSourceSpec(*video_stream_config_, demand.name)
                                      : resolveRosVideoTopicSpec(*video_stream_config_, demand.name, interface_type);
      stream_key = stream_spec.stream_key;
      std::string track_name;
      try {
        track_name = videoRegistry().start(stream_spec);
      } catch (const std::exception & exc) {
        throw StreamUnavailableError(exc.what());
      }
      subscription.runtime = VideoRuntime{std::move(track_name), std::move(stream_spec)};
    } else {
      auto data = std::shared_ptr<DataStreamInstance>(new DataStreamInstance(
        demand.name, interface_type, node_, room_connection_, *this, message_callback_gate_, subscription_qos_config_));
      data->setIntervalMs(appliedIntervalMs(subscription.leases));
      data->subscribe();
      subscription.runtime = std::move(data);
    }
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_renew_failed")
      .field("resource", demand.name)
      .field("kind", wire::subscriptions::targetKindString(demand.kind))
      .field("requester_identity", requester_identity)
      .fieldIf(stream_key.has_value(), "stream_key", stream_key.value_or(""))
      .field("error", exc.what())
      .warn();
    throw;
  }

  const std::string & resource = subscription.name;
  const SubscriptionTargetKind target_kind = subscription.target_kind;
  // Make the shared subscription visible before starting a data-track publish: completion
  // callbacks resolve back into the registry by deterministic track name via
  // findDataByTrackName().
  auto [subscription_it, inserted] = subscriptions_.emplace(
    std::string(wire::subscriptions::targetKindString(demand.kind)) + ":" + demand.name, std::move(subscription));
  (void)inserted;

  if (auto * data = dataInstance(subscription_it->second)) {
    const std::size_t publish_generation = registry_generation_.load();
    data->start(publish_generation);
  }

  SubscriptionStatus status = statusFor(subscription_it->second);
  LogEvent event(kLogger, "subscription_created");
  event.field("resource", resource)
    .field("kind", wire::subscriptions::targetKindString(target_kind))
    .field("delivery", wire::subscriptions::deliveryKindString(status.delivery_kind))
    .field("requester_identity", requester_identity);
  if (const auto * video = std::get_if<VideoRuntime>(&subscription_it->second.runtime)) {
    event.field("stream_key", video->stream_spec.stream_key).field("track_name", video->track_name);
  } else if (const auto * data = dataInstance(subscription_it->second)) {
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
  const std::string canonical_topic = normalizeRosResourceName(topic);
  return renewSubscription(
    requester_identity,
    SubscriptionDemand{SubscriptionTargetKind::Topic, canonical_topic, preferred_interval_ms},
    expiry);
}

void SubscriptionRegistry::queueDataTrackRepublish(const std::string & requester_identity, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return;
  }
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }
  const std::size_t current_generation = registry_generation_.load();
  if (generation != current_generation) {
    LogEvent(kLogger, "data_track_republish_queue_skipped")
      .field("requester_identity", requester_identity)
      .field("reason", "stale_generation")
      .field("observed_generation", generation)
      .field("current_generation", current_generation)
      .info();
    return;
  }

  for (const auto & [subscription_key, subscription] : subscriptions_) {
    (void)subscription_key;
    const auto * data = dataInstance(subscription);
    if (data == nullptr) {
      continue;
    }
    if (data->state() != DataStreamInstance::State::kPublished) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    // The republish queue is keyed only by requester. Once any currently published data track
    // proves this requester still owns a live lease, republishDataTracks() will sweep the rest.
    if (republish_requesters_.insert(requester_identity).second) {
      LogEvent(kLogger, "data_track_republish_queued")
        .field("resource", subscription.name)
        .field("track_name", data->trackName())
        .field("requester_identity", requester_identity)
        .field("reason", "participant_disconnected")
        .info();
    }
    return;
  }
}

void SubscriptionRegistry::republishDataTracks(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }
  if (republish_requesters_.erase(requester_identity) == 0U) {
    return;
  }

  const std::size_t publish_generation = registry_generation_.load();
  for (auto & [subscription_key, subscription] : subscriptions_) {
    (void)subscription_key;
    auto * data = dataInstance(subscription);
    if (data == nullptr) {
      continue;
    }
    if (data->state() != DataStreamInstance::State::kPublished) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    LogEvent(kLogger, "data_track_republish")
      .field("resource", subscription.name)
      .field("track_name", data->trackName())
      .field("requester_identity", requester_identity)
      .info();
    data->republish(publish_generation);
  }
}

void SubscriptionRegistry::revokeRequesterLeases(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }

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

SubscriptionRegistry::SharedSubscription * SubscriptionRegistry::findSubscription(
  SubscriptionTargetKind kind, const std::string & name)
{
  auto it = subscriptions_.find(std::string(wire::subscriptions::targetKindString(kind)) + ":" + name);
  return it == subscriptions_.end() ? nullptr : &it->second;
}

const SubscriptionRegistry::SharedSubscription * SubscriptionRegistry::findSubscription(
  SubscriptionTargetKind kind, const std::string & name) const
{
  const auto it = subscriptions_.find(std::string(wire::subscriptions::targetKindString(kind)) + ":" + name);
  return it == subscriptions_.end() ? nullptr : &it->second;
}

void SubscriptionRegistry::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }
  LogEvent(kLogger, "subscription_registry_reset_begin").field("subscription_count", subscriptions_.size()).info();

  const std::size_t callback_generation = message_callback_gate_.close();
  republish_requesters_.clear();
  clearSubscriptions();
  // Reopen only after old-session teardown completes so new callbacks cannot interleave with it.
  message_callback_gate_.open(callback_generation);
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
  if (auto * data = dataInstance(it->second)) {
    return data->completePublish(generation);
  }
  return false;
}

std::size_t SubscriptionRegistry::generation() const
{
  return registry_generation_.load();
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
  if (auto * data = dataInstance(it->second)) {
    data->failPublish();
  }
}

VideoStreamRegistry & SubscriptionRegistry::videoRegistry() const
{
  if (video_stream_registry_ == nullptr) {
    throw StreamUnavailableError("Video stream registry is unavailable.");
  }

  return *video_stream_registry_;
}

DataStreamInstance * SubscriptionRegistry::dataInstance(SharedSubscription & subscription)
{
  auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&subscription.runtime);
  return data == nullptr ? nullptr : data->get();
}

const DataStreamInstance * SubscriptionRegistry::dataInstance(const SharedSubscription & subscription)
{
  const auto * data = std::get_if<std::shared_ptr<DataStreamInstance>>(&subscription.runtime);
  return data == nullptr ? nullptr : data->get();
}

SubscriptionStatus SubscriptionRegistry::statusFor(const SharedSubscription & subscription)
{
  SubscriptionStatus status;
  status.kind = subscription.target_kind;
  status.name = subscription.name;
  status.interface_type = subscription.interface_type;
  if (const auto * data = dataInstance(subscription)) {
    status.delivery_kind = SubscriptionDeliveryKind::kData;
    if (data->state() == DataStreamInstance::State::kPending || data->state() == DataStreamInstance::State::kPublished)
    {
      status.track_name = data->trackName();
    }
    status.applied_interval_ms = data->intervalMs();
    return status;
  }

  const auto & video = std::get<VideoRuntime>(subscription.runtime);
  status.delivery_kind = SubscriptionDeliveryKind::kVideo;
  status.track_name = video.track_name;
  status.degraded_reason = video.stream_spec.degraded_reason.value_or("");
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

void SubscriptionRegistry::removeLeasesIf(
  const LeasePredicate & should_remove, LeaseRemovalReason reason, Clock::time_point reference_time)
{
  const char * removal_reason = "unknown";
  switch (reason) {
    case LeaseRemovalReason::kParticipantDisconnected:
      removal_reason = "participant_disconnected";
      break;
    case LeaseRemovalReason::kLeaseExpired:
      removal_reason = "lease_expired";
      break;
  }

  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto & subscription = it->second;
    bool removed_any = false;

    for (auto req_it = subscription.leases.begin(); req_it != subscription.leases.end();) {
      const auto & requester_identity = req_it->first;
      const auto & lease = req_it->second;
      if (!should_remove(requester_identity, lease)) {
        ++req_it;
        continue;
      }

      const auto remaining_requesters = subscription.leases.size() - 1U;
      const auto delta_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(lease.expiry - reference_time).count();
      if (remaining_requesters > 0U) {
        LogEvent(kLogger, "requester_lease_removed")
          .field("resource", subscription.name)
          .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
          .field("requester_identity", requester_identity)
          .field("reason", removal_reason)
          .field("remaining_requesters", remaining_requesters)
          .fieldIf(reason == LeaseRemovalReason::kLeaseExpired, "expired_by_ms", static_cast<long>(-delta_ms))
          .fieldIf(reason != LeaseRemovalReason::kLeaseExpired, "expires_in_ms", static_cast<long>(delta_ms))
          .info();
      }

      removed_any = true;
      republish_requesters_.erase(requester_identity);
      req_it = subscription.leases.erase(req_it);
    }

    if (!removed_any) {
      ++it;
      continue;
    }

    if (subscription.leases.empty()) {
      if (const auto * data = dataInstance(subscription)) {
        LogEvent event(kLogger, "subscription_pruned");
        event.field("resource", subscription.name)
          .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
          .field("reason", removal_reason)
          .field("track_name", data->trackName());
        event.info();
      } else {
        const auto & video = std::get<VideoRuntime>(subscription.runtime);
        LogEvent event(kLogger, "subscription_pruned");
        event.field("resource", subscription.name)
          .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
          .field("reason", removal_reason)
          .field("stream_key", video.stream_spec.stream_key)
          .field("track_name", video.track_name);
        event.info();
      }
      // `subscription_pruned` already captures this lease-driven teardown boundary.
      destroyRuntime(subscription, false);
      it = subscriptions_.erase(it);
      continue;
    }

    if (auto * data = dataInstance(subscription)) {
      data->setIntervalMs(appliedIntervalMs(subscription.leases));
    }
    ++it;
  }
}

SubscriptionRegistry::SubscriptionMap::iterator SubscriptionRegistry::findDataByTrackName(
  const std::string & track_name)
{
  // Registry storage is keyed by canonical subscription target, but data publish callbacks only
  // identify the deterministic LiveKit track name.
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    auto & subscription = it->second;
    auto * data = dataInstance(subscription);
    if (data == nullptr) {
      continue;
    }
    if (data->trackName() != track_name) {
      continue;
    }
    return it;
  }
  return subscriptions_.end();
}

void SubscriptionRegistry::destroyRuntime(SharedSubscription & subscription, bool log_destroy)
{
  if (auto * data = dataInstance(subscription)) {
    if (log_destroy) {
      LogEvent event(kLogger, "subscription_destroyed");
      event.field("resource", subscription.name)
        .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
        .field("track_name", data->trackName());
      event.info();
    }
    data->shutdown();
    subscription.runtime = std::shared_ptr<DataStreamInstance>{};
    // Track names are deterministic per topic, so destroying a data subscription must advance the
    // generation before any delayed publish/disconnect callback can target the replacement subscription.
    registry_generation_.fetch_add(1);
    return;
  }

  const auto & video = std::get<VideoRuntime>(subscription.runtime);
  if (log_destroy) {
    LogEvent event(kLogger, "subscription_destroyed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("stream_key", video.stream_spec.stream_key)
      .field("track_name", video.track_name);
    event.info();
  }
  videoRegistry().stop(video.stream_spec.stream_key);
}

void SubscriptionRegistry::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  LogEvent(kLogger, "subscription_registry_shutdown_begin").field("subscription_count", subscriptions_.size()).info();

  (void)message_callback_gate_.close();
  clearSubscriptions();
}

void SubscriptionRegistry::clearSubscriptions()
{
  // Remove visible registry state first so late publish/disconnect callbacks cannot resolve a
  // soon-to-be-destroyed subscription while teardown is running.
  auto owned_subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  republish_requesters_.clear();

  for (auto & subscription : owned_subscriptions) {
    destroyRuntime(subscription.second);
  }
  // Ensure generation advances even if no data subscriptions existed.
  registry_generation_.fetch_add(1);
}

}  // namespace livekit_ros2_bridge
