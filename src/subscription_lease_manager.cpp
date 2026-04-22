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
#include <exception>
#include <optional>
#include <utility>
#include <vector>

#include "data_track_publisher.hpp"
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
constexpr const char * kLeaseExpiredReason = "lease_expired";

}  // namespace

SubscriptionLeaseManager::SubscriptionLeaseManager(
  SubscriptionNodeInterfaces interfaces,
  RoomConnection & room_connection,
  AccessPolicy access_policy,
  VideoStreamRegistry & video_stream_registry,
  const SubscriptionQosConfig * qos_config,
  Clock::duration heartbeat_lease_duration)
: interfaces_(std::move(interfaces))
, room_connection_(room_connection)
, access_policy_(std::move(access_policy))
, video_stream_registry_(video_stream_registry)
, qos_config_(qos_config)
, heartbeat_lease_duration_(heartbeat_lease_duration)
{}

SubscriptionLeaseManager::SubscriptionLeaseManager(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  AccessPolicy access_policy,
  VideoStreamRegistry & video_stream_registry,
  const SubscriptionQosConfig * qos_config,
  Clock::duration heartbeat_lease_duration)
: SubscriptionLeaseManager(
    makeRosNodeInterfaces(node).subscription(),
    room_connection,
    std::move(access_policy),
    video_stream_registry,
    qos_config,
    heartbeat_lease_duration)
{}

SubscriptionLeaseManager::~SubscriptionLeaseManager()
{
  shutdown();
}

void SubscriptionLeaseManager::handleHeartbeat(
  const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat)
{
  const auto resolved_requester_identity = resolveIdentity(requester_identity, heartbeat.session_id);
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
    // `other_video` targets name bridge-owned config entries rather than ROS graph
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
      statuses.emplace_back(ensure(resolved_identity, demand, expiry));
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
  republishTracks(resolved_identity);

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
  packet.topic = wire::protocol::kBridgeStatusTopic;

  try {
    room_connection_.publishPacket(packet);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", resolved_identity)
      .fieldOr("session_id", heartbeat.session_id.value_or(""), "<absent>")
      .field("error", exc.what())
      .warnThrottle(*interfaces_.clock, kLogThrottle);
  }
}

std::optional<std::string> SubscriptionLeaseManager::resolveIdentity(
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
        .warnThrottle(*interfaces_.clock, kLogThrottle);

      return std::nullopt;
    }

    LogEvent(kLogger, "heartbeat_client_session_fallback")
      .field("requester_identity", it->second.requester_identity)
      .fieldOr("session_id", session_id.value_or(""), "<absent>")
      .warnThrottle(*interfaces_.clock, kLogThrottle);

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

DataTrackPublisher & SubscriptionLeaseManager::requireDataPublisher(const Subscription & subscription) const
{
  if (subscription.data_publisher == nullptr) {
    throw std::logic_error("data subscription invariant violated: data publisher is required");
  }

  return *subscription.data_publisher;
}

SubscriptionStatus SubscriptionLeaseManager::ensure(
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

  if (auto * subscription = find(demand.kind, demand.name)) {
    return renew(*subscription, requester_identity, lease);
  }

  return create(demand, requester_identity, lease);
}

SubscriptionStatus SubscriptionLeaseManager::renew(
  Subscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    const bool had_requester = subscription.leases.find(requester_identity) != subscription.leases.end();
    subscription.leases[requester_identity] = lease;

    if (isVideoSubscription(subscription)) {
      video_stream_registry_.start(subscription.target_kind, subscription.name, subscription.interface_type);
      return status(subscription);
    }

    auto & data_publisher = requireDataPublisher(subscription);
    const bool was_published = data_publisher.isPublished();
    data_publisher.setIntervalMs(appliedIntervalMs(subscription.leases));

    if (!had_requester && was_published) {
      // A new requester can receive subscription status before LiveKit surfaces the existing
      // published data track to that participant session, so queue one republish when the
      // requester first joins. The fresh lease was inserted just above, so only published
      // state matters here.
      republish_requesters_.insert(requester_identity);
    }

    if (!was_published) {
      data_publisher.publish();
    }
  } catch (...) {
    LogEvent(kLogger, "subscription_renew_failed")
      .field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();

    throw;
  }

  return status(subscription);
}

SubscriptionStatus SubscriptionLeaseManager::create(
  const SubscriptionDemand & demand, const std::string & requester_identity, const Lease & lease)
{
  Subscription subscription;
  subscription.target_kind = demand.kind;
  subscription.name = demand.name;

  try {
    if (demand.kind == SubscriptionTargetKind::Topic) {
      subscription.interface_type =
        requireSingleInterfaceType(interfaces_.graph->get_topic_names_and_types(), demand.name, "topic");
    }
    subscription.leases.emplace(requester_identity, lease);

    if (isVideoSubscription(subscription)) {
      video_stream_registry_.start(demand.kind, demand.name, subscription.interface_type);
    } else {
      subscription.data_publisher = DataTrackPublisher::create(
        demand.name, subscription.interface_type, interfaces_, room_connection_, qos_config_);
      subscription.data_publisher->setIntervalMs(appliedIntervalMs(subscription.leases));
      subscription.data_publisher->publish();
    }
  } catch (...) {
    LogEvent(kLogger, "subscription_renew_failed")
      .field("resource", demand.name)
      .field("kind", wire::subscriptions::targetKindString(demand.kind))
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();
    throw;
  }

  // TODO: put this before the start calls
  auto subscription_it =
    subscriptions_
      .emplace(
        std::string(wire::subscriptions::targetKindString(demand.kind)) + ":" + demand.name, std::move(subscription))
      .first;

  SubscriptionStatus result = status(subscription_it->second);
  LogEvent(kLogger, "subscription_created")
    .field("resource", demand.name)
    .field("kind", wire::subscriptions::targetKindString(demand.kind))
    .field("delivery", wire::subscriptions::deliveryKindString(result.delivery_kind))
    .field("requester_identity", requester_identity)
    .info();

  return result;
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
    if (isVideoSubscription(subscription)) {
      continue;
    }

    const auto & data_publisher = requireDataPublisher(subscription);
    if (!data_publisher.isPublished()) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    // The republish queue is keyed only by requester. Once any currently published data track
    // proves this requester still owns a live lease, republishTracks() will sweep the rest.
    republish_requesters_.insert(requester_identity);

    return;
  }
}

void SubscriptionLeaseManager::republishTracks(const std::string & requester_identity)
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
    if (isVideoSubscription(subscription)) {
      continue;
    }

    auto & data_publisher = requireDataPublisher(subscription);
    if (!data_publisher.isPublished()) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    LogEvent(kLogger, "data_track_republish")
      .field("resource", subscription.name)
      .field("track_name", data_publisher.name())
      .field("requester_identity", requester_identity)
      .info();
    data_publisher.republish();
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

  pruneExpiredSubscriptionLeases(now);
}

SubscriptionLeaseManager::Subscription * SubscriptionLeaseManager::find(
  SubscriptionTargetKind kind, const std::string & name)
{
  auto it = subscriptions_.find(std::string(wire::subscriptions::targetKindString(kind)) + ":" + name);
  return it == subscriptions_.end() ? nullptr : &it->second;
}

const SubscriptionLeaseManager::Subscription * SubscriptionLeaseManager::find(
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
    destroy(subscription);
  }
}

SubscriptionStatus SubscriptionLeaseManager::status(const Subscription & subscription) const
{
  SubscriptionStatus status;
  status.kind = subscription.target_kind;
  status.name = subscription.name;
  status.interface_type = subscription.interface_type;

  if (!isVideoSubscription(subscription)) {
    const auto & data_publisher = requireDataPublisher(subscription);
    status.delivery_kind = SubscriptionDeliveryKind::kData;
    if (data_publisher.isPublished()) {
      status.track_name = data_publisher.name();
    }
    status.applied_interval_ms = data_publisher.intervalMs();
    return status;
  }

  const auto video =
    video_stream_registry_.find(subscription.target_kind, subscription.name, subscription.interface_type);
  if (!video.has_value()) {
    throw std::logic_error("video subscription invariant violated: video stream is required");
  }
  status.delivery_kind = SubscriptionDeliveryKind::kVideo;
  status.track_name = video->track_name;
  status.degraded_reason = video->degraded_reason;
  return status;
}

bool SubscriptionLeaseManager::isVideoSubscription(const Subscription & subscription)
{
  return subscription.target_kind == SubscriptionTargetKind::OtherVideo ||
         classifyRosVideoIngestMode(subscription.interface_type).has_value();
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

std::vector<SubscriptionLeaseManager::ExpiredLeaseRemoval> SubscriptionLeaseManager::collectExpiredLeaseRemovals(
  const Subscription & subscription, Clock::time_point reference_time)
{
  std::vector<ExpiredLeaseRemoval> removals;
  for (const auto & [requester_identity, lease] : subscription.leases) {
    if (reference_time < lease.expiry) {
      continue;
    }

    removals.push_back(ExpiredLeaseRemoval{requester_identity, lease.expiry});
  }
  return removals;
}

void SubscriptionLeaseManager::applyExpiredLeaseRemovals(
  Subscription & subscription, const std::vector<ExpiredLeaseRemoval> & removals, Clock::time_point reference_time)
{
  for (const auto & removal : removals) {
    const auto remaining_requesters = subscription.leases.size() - 1U;
    const auto delta_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(removal.expiry - reference_time).count();
    if (remaining_requesters > 0U) {
      LogEvent(kLogger, "requester_lease_removed")
        .field("resource", subscription.name)
        .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
        .field("requester_identity", removal.requester_identity)
        .field("reason", kLeaseExpiredReason)
        .field("remaining_requesters", remaining_requesters)
        .field("expired_by_ms", static_cast<long>(-delta_ms))
        .info();
    }

    republish_requesters_.erase(removal.requester_identity);
    subscription.leases.erase(removal.requester_identity);
  }
}

void SubscriptionLeaseManager::refreshDataSubscriptionInterval(const Subscription & subscription)
{
  if (isVideoSubscription(subscription) || subscription.leases.empty()) {
    return;
  }

  requireDataPublisher(subscription).setIntervalMs(appliedIntervalMs(subscription.leases));
}

void SubscriptionLeaseManager::pruneExpiredSubscriptionLeases(Clock::time_point reference_time)
{
  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto & subscription = it->second;
    const auto removals = collectExpiredLeaseRemovals(subscription, reference_time);
    if (removals.empty()) {
      ++it;
      continue;
    }

    applyExpiredLeaseRemovals(subscription, removals, reference_time);

    if (!subscription.leases.empty()) {
      refreshDataSubscriptionInterval(subscription);
      ++it;
      continue;
    }

    LogEvent(kLogger, "subscription_pruned")
      .field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("reason", kLeaseExpiredReason)
      .info();
    // `subscription_pruned` already captures this lease-driven teardown boundary.
    destroy(subscription, false);
    it = subscriptions_.erase(it);
  }
}

void SubscriptionLeaseManager::destroy(Subscription & subscription, bool log_destroy)
{
  if (!isVideoSubscription(subscription)) {
    auto & data_publisher = requireDataPublisher(subscription);
    if (log_destroy) {
      LogEvent(kLogger, "subscription_destroyed")
        .field("resource", subscription.name)
        .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
        .field("track_name", data_publisher.name())
        .info();
    }
    subscription.data_publisher.reset();
    return;
  }

  const auto video =
    video_stream_registry_.find(subscription.target_kind, subscription.name, subscription.interface_type);
  if (!video.has_value()) {
    throw std::logic_error("video subscription invariant violated: video stream is required");
  }
  if (log_destroy) {
    LogEvent(kLogger, "subscription_destroyed")
      .field("resource", subscription.name)
      .field("kind", wire::subscriptions::targetKindString(subscription.target_kind))
      .field("stream_key", video->stream_key)
      .field("track_name", video->track_name)
      .info();
  }
  video_stream_registry_.stop(subscription.target_kind, subscription.name, subscription.interface_type);
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
    destroy(subscription);
  }
}

}  // namespace livekit_ros2_bridge
