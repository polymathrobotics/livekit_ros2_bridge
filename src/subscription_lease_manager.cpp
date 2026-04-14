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

#include "subscription_lease_manager.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "video_stream_registry.hpp"
#include "video_stream_spec.hpp"
#include "wire/protocol.hpp"
#include "wire/subscriptions.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("subscription_lease_manager");

}  // namespace

SubscriptionLeaseManager::SubscriptionLeaseManager(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  AccessPolicy access_policy,
  rclcpp::Clock::SharedPtr clock,
  DataStreamRegistry & data_stream_registry,
  VideoStreamRegistry & video_stream_registry,
  Clock::duration heartbeat_lease_duration)
: node_(node)
, room_connection_(room_connection)
, access_policy_(std::move(access_policy))
, clock_(std::move(clock))
, data_stream_registry_(data_stream_registry)
, video_stream_registry_(video_stream_registry)
, heartbeat_lease_duration_(heartbeat_lease_duration)
{}

void SubscriptionLeaseManager::handleHeartbeat(
  const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat)
{
  const auto resolved_requester_identity = resolveRequesterIdentity(requester_identity, heartbeat.session_id);
  if (!resolved_requester_identity.has_value()) {
    return;
  }
  const auto & resolved_identity = *resolved_requester_identity;

  const auto expiry = Clock::now() + heartbeat_lease_duration_;
  if (heartbeat.session_id.has_value()) {
    auto [it, inserted] = session_leases_.try_emplace(*heartbeat.session_id, SessionLease{resolved_identity, expiry});
    if (!inserted && it->second.requester_identity != resolved_identity) {
      throw std::logic_error("session lease invariant violated: session_id must resolve to one requester");
    }

    it->second.expiry = expiry;
  }

  std::vector<SubscriptionReportedStatus> statuses;
  statuses.reserve(heartbeat.subscriptions.size());

  for (const auto & demand : heartbeat.subscriptions) {
    // `configured_source` targets name bridge-owned config entries rather than ROS graph
    // resources, so subscribe ACLs apply only to true ROS topic subscriptions here.
    if (demand.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, demand.name))
    {
      statuses.emplace_back(
        SubscriptionErrorStatus{
          demand.kind,
          demand.name,
          SubscriptionStatusErrorReason::kForbidden,
          "ROS topic '" + demand.name + "' not permitted.",
        });
      continue;
    }

    try {
      statuses.emplace_back(renewSubscription(resolved_identity, demand, expiry));
    } catch (const std::exception & exc) {
      statuses.emplace_back(
        SubscriptionErrorStatus{
          demand.kind,
          demand.name,
          SubscriptionStatusErrorReason::kNotFound,
          exc.what(),
        });
    }
  }

  // A page refresh can reuse the requester identity before the old lease expires, but the
  // rejoined participant still needs a fresh data-track publication because the previous one
  // belonged to the disconnected participant_session.
  republishDataTracks(resolved_identity);

  // A heartbeat may exist only to bind or renew the client-session lease. In that case the wire
  // contract does not send an empty status envelope back.
  if (statuses.empty()) {
    return;
  }

  const std::string body =
    wire::subscriptions::serializeStatuses(statuses, heartbeat.session_id, std::optional<Clock::time_point>{expiry})
      .dump();
  OutgoingPacket packet;
  packet.payload = std::vector<std::uint8_t>(body.begin(), body.end());
  packet.recipient_identities = {resolved_identity};
  packet.topic = wire::protocol::kControlSubscriptionsStatus;

  try {
    room_connection_.publishPacket(packet);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", resolved_identity)
      .fieldOr("session_id", heartbeat.session_id.value_or(""), "<absent>")
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
  }
}

std::optional<std::string> SubscriptionLeaseManager::resolveRequesterIdentity(
  const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  if (requester_identity.empty()) {
    // LiveKit should normally attach the requester identity to user-data packets. If it does not, a
    // known wire session_id is enough to treat the heartbeat as belonging to the same authenticated
    // browser tab and renew that client-session lease instead of dropping it.
    const auto it = session_id.has_value() ? session_leases_.find(*session_id) : session_leases_.end();
    if (it == session_leases_.end()) {
      LogEvent(kLogger, "heartbeat_dropped")
        .field("reason", "anonymous_requester_without_resolvable_client_session")
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
        .warnThrottle(*clock_, kLogThrottle);

      return std::nullopt;
    }

    LogEvent(kLogger, "heartbeat_client_session_fallback")
      .field("requester_identity", it->second.requester_identity)
      .fieldOr("session_id", session_id.value_or(""), "<absent>")
      .warnThrottle(*clock_, kLogThrottle);

    return it->second.requester_identity;
  }

  if (!session_id.has_value()) {
    return requester_identity;
  }

  const auto it = session_leases_.find(*session_id);
  if (it != session_leases_.end() && it->second.requester_identity != requester_identity) {
    if (const std::size_t count = conflict_throttle_.recordAndTakePendingCount(); count > 0U) {
      LogEvent(kLogger, "heartbeat_client_session_conflict")
        .field("requester_identity", requester_identity)
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
        .field("existing_requester_identity", it->second.requester_identity)
        .field("count", count)
        .warn();
    }

    return std::nullopt;
  }

  return requester_identity;
}

SubscriptionStatus SubscriptionLeaseManager::renewSubscription(
  const std::string & requester_identity, const SubscriptionDemand & demand, Clock::time_point expiry)
{
  if (is_shutdown_.load()) {
    throw std::runtime_error("Subscription registry is shut down.");
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

SubscriptionStatus SubscriptionLeaseManager::renewExistingSubscription(
  SharedSubscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    const bool had_requester = subscription.leases.find(requester_identity) != subscription.leases.end();
    subscription.leases[requester_identity] = lease;

    if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
      const auto * data = data_stream_registry_.find(subscription.name);
      if (data == nullptr) {
        throw std::logic_error("data subscription invariant violated: data stream is required");
      }
      const DataStreamInstance::State state = data->state();

      data_stream_registry_.setIntervalMs(subscription.name, appliedIntervalMs(subscription.leases));

      const bool is_published = state == DataStreamInstance::State::kPublished;
      if (!had_requester && is_published) {
        // A new requester can receive subscription status before LiveKit surfaces the existing
        // published data track to that participant session, so queue one republish when the
        // requester first joins. The fresh lease was inserted just above, so only published
        // state matters here.
        republish_requesters_.insert(requester_identity);
      }

      if (state == DataStreamInstance::State::kNone || state == DataStreamInstance::State::kFailed) {
        data_stream_registry_.start(subscription.name);
      }

      return statusFor(subscription);
    }

    const VideoStreamRequest request{subscription.target_kind, subscription.name, subscription.interface_type};
    video_stream_registry_.start(request);
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "subscription_renew_failed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("requester_identity", requester_identity);
    if (subscription.delivery_kind == SubscriptionDeliveryKind::kVideo) {
      const VideoStreamRequest request{subscription.target_kind, subscription.name, subscription.interface_type};
      try {
        const auto active_video = video_stream_registry_.find(request);
        const auto video = active_video.has_value() ? *active_video : video_stream_registry_.resolve(request);
        event.field("stream_key", video.stream_key).field("track_name", video.track_name);
      } catch (const std::exception &) {}
    } else if (const auto * data = data_stream_registry_.find(subscription.name)) {
      event.field("track_name", data->trackName());
    }
    event.field("error", exc.what()).warn();
    throw;
  }

  return statusFor(subscription);
}

SubscriptionStatus SubscriptionLeaseManager::createSubscription(
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
      subscription.delivery_kind = SubscriptionDeliveryKind::kVideo;
      const VideoStreamRequest request{demand.kind, demand.name, interface_type};
      stream_key = video_stream_registry_.resolve(request).stream_key;
      video_stream_registry_.start(request);
    } else {
      data_stream_registry_.create(demand.name, interface_type);
      data_stream_registry_.setIntervalMs(demand.name, appliedIntervalMs(subscription.leases));
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
  // callbacks reconcile against DataStreamRegistry by deterministic track name.
  auto [subscription_it, inserted] = subscriptions_.emplace(
    std::string(wire::subscriptions::targetKindString(demand.kind)) + ":" + demand.name, std::move(subscription));
  (void)inserted;

  if (subscription_it->second.delivery_kind != SubscriptionDeliveryKind::kVideo) {
    data_stream_registry_.start(subscription_it->second.name);
  }

  SubscriptionStatus status = statusFor(subscription_it->second);
  LogEvent event(kLogger, "subscription_created");
  event.field("resource", resource)
    .field("kind", wire::subscriptions::targetKindString(target_kind))
    .field("delivery", wire::subscriptions::deliveryKindString(status.delivery_kind))
    .field("requester_identity", requester_identity);
  if (subscription_it->second.delivery_kind == SubscriptionDeliveryKind::kVideo) {
    const VideoStreamRequest request{
      subscription_it->second.target_kind, subscription_it->second.name, subscription_it->second.interface_type};
    const auto video = video_stream_registry_.find(request);
    if (!video.has_value()) {
      throw std::logic_error("video subscription invariant violated: video stream is required");
    }
    event.field("stream_key", video->stream_key).field("track_name", video->track_name);
  } else if (const auto * data = data_stream_registry_.find(subscription_it->second.name)) {
    event.field("track_name", data->trackName());
  }
  event.info();
  return status;
}

void SubscriptionLeaseManager::onRemoteParticipantDisconnected(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  if (requester_identity.empty()) {
    throw std::invalid_argument("requester_identity is required");
  }

  for (const auto & [subscription_key, subscription] : subscriptions_) {
    (void)subscription_key;
    if (subscription.delivery_kind == SubscriptionDeliveryKind::kVideo) {
      continue;
    }

    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
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

void SubscriptionLeaseManager::republishDataTracks(const std::string & requester_identity)
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

  for (auto & [subscription_key, subscription] : subscriptions_) {
    (void)subscription_key;
    if (subscription.delivery_kind == SubscriptionDeliveryKind::kVideo) {
      continue;
    }

    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
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
    data_stream_registry_.republish(subscription.name);
  }
}

void SubscriptionLeaseManager::pruneExpiredLeases()
{
  if (is_shutdown_.load()) {
    return;
  }

  const auto now = Clock::now();
  for (auto it = session_leases_.begin(); it != session_leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    it = session_leases_.erase(it);
  }

  removeLeasesIf([now](const std::string &, const Lease & lease) { return now >= lease.expiry; }, now);
}

SubscriptionLeaseManager::SharedSubscription * SubscriptionLeaseManager::findSubscription(
  SubscriptionTargetKind kind, const std::string & name)
{
  auto it = subscriptions_.find(std::string(wire::subscriptions::targetKindString(kind)) + ":" + name);
  return it == subscriptions_.end() ? nullptr : &it->second;
}

const SubscriptionLeaseManager::SharedSubscription * SubscriptionLeaseManager::findSubscription(
  SubscriptionTargetKind kind, const std::string & name) const
{
  const auto it = subscriptions_.find(std::string(wire::subscriptions::targetKindString(kind)) + ":" + name);
  return it == subscriptions_.end() ? nullptr : &it->second;
}

void SubscriptionLeaseManager::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }
  LogEvent(kLogger, "subscription_registry_reset_begin").field("subscription_count", subscriptions_.size()).info();
  auto owned_subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  republish_requesters_.clear();

  for (auto & [subscription_key, subscription] : owned_subscriptions) {
    (void)subscription_key;
    if (subscription.delivery_kind == SubscriptionDeliveryKind::kVideo) {
      continue;
    }

    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
    }

    LogEvent event(kLogger, "subscription_destroyed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("track_name", data->trackName());
    event.info();
  }

  data_stream_registry_.resetSessionState();

  for (auto & [subscription_key, subscription] : owned_subscriptions) {
    (void)subscription_key;
    if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
      continue;
    }
    destroyRuntime(subscription);
  }
}

SubscriptionStatus SubscriptionLeaseManager::statusFor(const SharedSubscription & subscription) const
{
  SubscriptionStatus status;
  status.kind = subscription.target_kind;
  status.name = subscription.name;
  status.interface_type = subscription.interface_type;

  if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
    }
    status.delivery_kind = SubscriptionDeliveryKind::kData;
    if (data->state() == DataStreamInstance::State::kPending || data->state() == DataStreamInstance::State::kPublished)
    {
      status.track_name = data->trackName();
    }
    status.applied_interval_ms = data->intervalMs();
    return status;
  }

  const VideoStreamRequest request{subscription.target_kind, subscription.name, subscription.interface_type};
  const auto video = video_stream_registry_.find(request);
  if (!video.has_value()) {
    throw std::logic_error("video subscription invariant violated: video stream is required");
  }
  status.delivery_kind = SubscriptionDeliveryKind::kVideo;
  status.track_name = video->track_name;
  status.degraded_reason = video->degraded_reason;
  return status;
}

int SubscriptionLeaseManager::appliedIntervalMs(const std::map<std::string, Lease> & leases)
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

void SubscriptionLeaseManager::removeLeasesIf(const LeasePredicate & should_remove, Clock::time_point reference_time)
{
  constexpr const char * kRemovalReason = "lease_expired";

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
          .field("reason", kRemovalReason)
          .field("remaining_requesters", remaining_requesters)
          .field("expired_by_ms", static_cast<long>(-delta_ms))
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
      if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
        const auto * data = data_stream_registry_.find(subscription.name);
        if (data == nullptr) {
          throw std::logic_error("data subscription invariant violated: data stream is required");
        }
        LogEvent event(kLogger, "subscription_pruned");
        event.field("resource", subscription.name)
          .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
          .field("reason", kRemovalReason)
          .field("track_name", data->trackName());
        event.info();
      } else {
        const VideoStreamRequest request{subscription.target_kind, subscription.name, subscription.interface_type};
        const auto video = video_stream_registry_.find(request);
        if (!video.has_value()) {
          throw std::logic_error("video subscription invariant violated: video stream is required");
        }
        LogEvent event(kLogger, "subscription_pruned");
        event.field("resource", subscription.name)
          .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
          .field("reason", kRemovalReason)
          .field("stream_key", video->stream_key)
          .field("track_name", video->track_name);
        event.info();
      }
      // `subscription_pruned` already captures this lease-driven teardown boundary.
      destroyRuntime(subscription, false);
      it = subscriptions_.erase(it);
      continue;
    }

    if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
      data_stream_registry_.setIntervalMs(subscription.name, appliedIntervalMs(subscription.leases));
    }
    ++it;
  }
}

void SubscriptionLeaseManager::destroyRuntime(SharedSubscription & subscription, bool log_destroy)
{
  if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
    }
    if (log_destroy) {
      LogEvent event(kLogger, "subscription_destroyed");
      event.field("resource", subscription.name)
        .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
        .field("track_name", data->trackName());
      event.info();
    }
    data_stream_registry_.stop(subscription.name);
    return;
  }

  const VideoStreamRequest request{subscription.target_kind, subscription.name, subscription.interface_type};
  const auto video = video_stream_registry_.find(request);
  if (!video.has_value()) {
    throw std::logic_error("video subscription invariant violated: video stream is required");
  }
  if (log_destroy) {
    LogEvent event(kLogger, "subscription_destroyed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("stream_key", video->stream_key)
      .field("track_name", video->track_name);
    event.info();
  }
  video_stream_registry_.stop(request);
}

void SubscriptionLeaseManager::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  LogEvent(kLogger, "subscription_registry_shutdown_begin").field("subscription_count", subscriptions_.size()).info();
  session_leases_.clear();
  auto owned_subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  republish_requesters_.clear();

  for (auto & [subscription_key, subscription] : owned_subscriptions) {
    (void)subscription_key;
    if (subscription.delivery_kind == SubscriptionDeliveryKind::kVideo) {
      continue;
    }

    const auto * data = data_stream_registry_.find(subscription.name);
    if (data == nullptr) {
      throw std::logic_error("data subscription invariant violated: data stream is required");
    }

    LogEvent event(kLogger, "subscription_destroyed");
    event.field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("track_name", data->trackName());
    event.info();
  }

  data_stream_registry_.shutdown();

  for (auto & [subscription_key, subscription] : owned_subscriptions) {
    (void)subscription_key;
    if (subscription.delivery_kind != SubscriptionDeliveryKind::kVideo) {
      continue;
    }
    destroyRuntime(subscription);
  }
}

}  // namespace livekit_ros2_bridge
