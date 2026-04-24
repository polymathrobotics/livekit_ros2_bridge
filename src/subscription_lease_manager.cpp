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
#include <stdexcept>
#include <utility>
#include <vector>

#include "data_track_publisher.hpp"
#include "protocol/constants.hpp"
#include "protocol/subscriptions_json.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("subscription_lease_manager");
constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr const char * kLeaseExpiredReason = "lease_expired";

const VideoStreamConfig & defaultVideoStreamConfig()
{
  static const VideoStreamConfig kDefaultConfig = makeDefaultVideoStreamConfig();
  return kDefaultConfig;
}

const char * targetKindLabel(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::OtherVideo:
      return "other_video";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

const char * deliveryKindLabel(SubscriptionDeliveryKind delivery)
{
  switch (delivery) {
    case SubscriptionDeliveryKind::Data:
      return "data";
    case SubscriptionDeliveryKind::Video:
      return "video";
  }

  throw std::invalid_argument("subscription delivery kind is invalid");
}

std::string makeSubscriptionKey(SubscriptionTargetKind kind, const std::string & name)
{
  const auto kind_string = targetKindLabel(kind);
  std::string key;
  key.reserve(std::char_traits<char>::length(kind_string) + 1U + name.size());
  key.append(kind_string);
  key.push_back(':');
  key.append(name);
  return key;
}

std::string normalizeDemandLookupName(SubscriptionTargetKind kind, const std::string & name)
{
  if (kind == SubscriptionTargetKind::Topic) {
    const std::string normalized_name = normalizeRosResourceName(name);
    if (normalized_name.empty()) {
      throw std::invalid_argument("Invalid ROS topic.");
    }
    return normalized_name;
  }

  if (kind == SubscriptionTargetKind::OtherVideo) {
    const std::string trimmed_name = trim(name);
    if (trimmed_name.empty()) {
      throw std::invalid_argument("Invalid other video name.");
    }
    return trimmed_name;
  }

  return name;
}

}  // namespace

SubscriptionLeaseManager::SubscriptionLeaseManager(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::Clock::SharedPtr clock,
  RoomConnection & room_connection,
  AccessPolicy access_policy,
  const SubscriptionQosConfig * qos_config,
  const VideoStreamConfig * video_stream_config,
  Clock::duration heartbeat_lease_duration)
: parameters_(std::move(parameters))
, topics_(std::move(topics))
, graph_(std::move(graph))
, clock_(std::move(clock))
, room_connection_(room_connection)
, access_policy_(std::move(access_policy))
, qos_config_(qos_config)
, video_stream_config_(video_stream_config)
, heartbeat_lease_duration_(heartbeat_lease_duration)
{}

SubscriptionLeaseManager::~SubscriptionLeaseManager()
{
  shutdown();
}

void SubscriptionLeaseManager::handleHeartbeatPayload(
  const std::string & requester_identity, const std::vector<std::uint8_t> & payload)
{
  std::optional<SubscriptionHeartbeat> heartbeat;
  try {
    heartbeat = protocol::subscriptions::parseHeartbeat(payload);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "packet_rejected")
      .field("reason", "invalid_heartbeat")
      .fieldOr("requester_identity", requester_identity)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
    return;
  }

  handleHeartbeat(requester_identity, *heartbeat);
}

void SubscriptionLeaseManager::startLeaseGcTimer(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers,
  SubmitToExecutorFunction submit_to_executor)
{
  lease_gc_timer_ = rclcpp::create_wall_timer(
    kLeaseGcInterval,
    [this, submit_to_executor = std::move(submit_to_executor)]() {
      submit_to_executor([this]() { pruneExpiredLeases(); });
    },
    nullptr,
    base.get(),
    timers.get());
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

  SubscriptionStatusReport report;
  report.session_id = heartbeat.session_id;
  report.lease_expiry = expiry;
  report.statuses.reserve(heartbeat.demands.size());

  for (const auto & demand : heartbeat.demands) {
    // `other_video` targets name bridge-owned config entries rather than ROS graph
    // resources, so subscribe ACLs apply only to true ROS topic subscriptions here.
    if (demand.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, demand.name))
    {
      report.statuses.emplace_back(
        SubscriptionErrorStatus{
          demand.kind,
          demand.name,
          SubscriptionStatusErrorReason::Forbidden,
          "ROS topic '" + demand.name + "' not permitted.",
        });
      continue;
    }

    try {
      report.statuses.emplace_back(ensure(resolved_identity, demand, expiry));
    } catch (const std::exception & exc) {
      report.statuses.emplace_back(
        SubscriptionErrorStatus{
          demand.kind,
          demand.name,
          SubscriptionStatusErrorReason::NotFound,
          exc.what(),
        });
    }
  }

  // A page refresh can reuse the requester identity before the old lease expires, but the
  // rejoined participant still needs a fresh data-track publication because the previous one
  // belonged to the disconnected participant_session.
  republishTracks(resolved_identity);

  // A heartbeat may exist only to bind or renew the client-session lease. In that case the protocol
  // contract does not send an empty status envelope back.
  if (report.statuses.empty()) {
    return;
  }

  const std::string body = protocol::subscriptions::serializeStatusReport(report);
  const std::vector<std::uint8_t> payload(body.begin(), body.end());
  const std::vector<std::string> destination_identities{resolved_identity};

  try {
    room_connection_.publishData(payload, true, destination_identities, protocol::kStatusTopic);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", resolved_identity)
      .fieldOr("session_id", heartbeat.session_id.value_or(""), "<absent>")
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
  }
}

std::optional<std::string> SubscriptionLeaseManager::resolveIdentity(
  const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  if (requester_identity.empty()) {
    // LiveKit should normally attach the requester identity to user-data packets. If it does not, a
    // known protocol session_id is enough to treat the heartbeat as belonging to the same authenticated
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

const VideoStreamConfig & SubscriptionLeaseManager::videoStreamConfig() const
{
  return video_stream_config_ == nullptr ? defaultVideoStreamConfig() : *video_stream_config_;
}

VideoStreamSpec SubscriptionLeaseManager::resolveVideoSpec(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return resolveRosVideoTopicSpec(videoStreamConfig(), name, interface_type);
    case SubscriptionTargetKind::OtherVideo:
      return resolveOtherVideoSourceSpec(videoStreamConfig(), name);
  }

  throw std::invalid_argument("video stream request kind is invalid");
}

std::shared_ptr<VideoTrackPublisher> SubscriptionLeaseManager::createVideoPublisher(const VideoStreamSpec & spec)
{
  return VideoTrackPublisher::create(parameters_, topics_, graph_, room_connection_, spec, qos_config_);
}

DataTrackPublisher & SubscriptionLeaseManager::requireDataPublisher(const Subscription & subscription) const
{
  if (subscription.data_publisher == nullptr) {
    throw std::logic_error("data subscription invariant violated: data publisher is required");
  }

  return *subscription.data_publisher;
}

VideoTrackPublisher & SubscriptionLeaseManager::requireVideoPublisher(const Subscription & subscription) const
{
  if (subscription.video_publisher == nullptr) {
    throw std::logic_error("video subscription invariant violated: video publisher is required");
  }

  return *subscription.video_publisher;
}

SubscriptionLeaseManager::ResolvedDemand SubscriptionLeaseManager::resolveDemand(
  const SubscriptionDemand & demand) const
{
  ResolvedDemand resolved;
  resolved.kind = demand.kind;
  resolved.name = normalizeDemandLookupName(demand.kind, demand.name);
  resolved.key = makeSubscriptionKey(demand.kind, resolved.name);

  if (demand.kind == SubscriptionTargetKind::Topic) {
    resolved.interface_type = requireSingleInterfaceType(graph_->get_topic_names_and_types(), resolved.name, "topic");
    if (!classifyRosVideoIngestMode(resolved.interface_type).has_value()) {
      return resolved;
    }
  }

  resolved.video_spec = resolveVideoSpec(demand.kind, resolved.name, resolved.interface_type);
  return resolved;
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
  const auto lookup_key = makeSubscriptionKey(demand.kind, normalizeDemandLookupName(demand.kind, demand.name));
  if (auto subscription_it = subscriptions_.find(lookup_key); subscription_it != subscriptions_.end()) {
    return renew(subscription_it->second, requester_identity, lease);
  }

  return create(resolveDemand(demand), requester_identity, lease);
}

SubscriptionStatus SubscriptionLeaseManager::renew(
  Subscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    const bool had_requester = subscription.leases.find(requester_identity) != subscription.leases.end();
    subscription.leases[requester_identity] = lease;

    if (isVideo(subscription)) {
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
      .field("kind", targetKindLabel(subscription.kind))
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();

    throw;
  }

  return status(subscription);
}

SubscriptionStatus SubscriptionLeaseManager::create(
  const ResolvedDemand & demand, const std::string & requester_identity, const Lease & lease)
{
  Subscription subscription;
  subscription.kind = demand.kind;
  subscription.name = demand.name;
  subscription.interface_type = demand.interface_type;

  try {
    subscription.leases.emplace(requester_identity, lease);

    if (demand.video_spec.has_value()) {
      subscription.video_publisher = createVideoPublisher(*demand.video_spec);
    } else {
      subscription.data_publisher = DataTrackPublisher::create(
        demand.name, subscription.interface_type, topics_, graph_, clock_, room_connection_, qos_config_);
      subscription.data_publisher->setIntervalMs(appliedIntervalMs(subscription.leases));
      subscription.data_publisher->publish();
    }
  } catch (...) {
    LogEvent(kLogger, "subscription_create_failed")
      .field("resource", demand.name)
      .field("kind", targetKindLabel(demand.kind))
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();
    throw;
  }

  auto subscription_it = subscriptions_.emplace(demand.key, std::move(subscription)).first;

  SubscriptionStatus result = status(subscription_it->second);
  LogEvent(kLogger, "subscription_created")
    .field("resource", subscription_it->second.name)
    .field("kind", targetKindLabel(demand.kind))
    .field("delivery", deliveryKindLabel(result.delivery))
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

  for (const auto & [key, subscription] : subscriptions_) {
    (void)key;
    if (isVideo(subscription)) {
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

  for (auto & [key, subscription] : subscriptions_) {
    (void)key;
    if (isVideo(subscription)) {
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

  pruneSubscriptionLeases(now);
}

SubscriptionLeaseManager::Subscription * SubscriptionLeaseManager::find(
  SubscriptionTargetKind kind, const std::string & name)
{
  auto it = subscriptions_.find(makeSubscriptionKey(kind, name));
  return it == subscriptions_.end() ? nullptr : &it->second;
}

const SubscriptionLeaseManager::Subscription * SubscriptionLeaseManager::find(
  SubscriptionTargetKind kind, const std::string & name) const
{
  const auto it = subscriptions_.find(makeSubscriptionKey(kind, name));
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

  for (auto & [key, subscription] : owned_subscriptions) {
    (void)key;
    destroy(subscription);
  }
}

SubscriptionStatus SubscriptionLeaseManager::status(const Subscription & subscription) const
{
  SubscriptionStatus status;
  status.kind = subscription.kind;
  status.name = subscription.name;
  status.interface_type = subscription.interface_type;

  if (!isVideo(subscription)) {
    const auto & data_publisher = requireDataPublisher(subscription);
    status.delivery = SubscriptionDeliveryKind::Data;
    if (data_publisher.isPublished()) {
      status.track_name = data_publisher.name();
    }
    status.interval_ms = data_publisher.intervalMs();
    return status;
  }

  const auto & video_spec = requireVideoPublisher(subscription).spec();
  status.delivery = SubscriptionDeliveryKind::Video;
  status.track_name = video_spec.track_name;
  status.degradation_reason = video_spec.degraded_reason.value_or("");
  return status;
}

bool SubscriptionLeaseManager::isVideo(const Subscription & subscription)
{
  return subscription.kind == SubscriptionTargetKind::OtherVideo ||
         classifyRosVideoIngestMode(subscription.interface_type).has_value();
}

int SubscriptionLeaseManager::appliedIntervalMs(const std::map<std::string, Lease> & leases)
{
  if (leases.empty()) {
    return 0;
  }

  int interval_ms = leases.begin()->second.preferred_interval_ms;
  for (const auto & [id, lease] : leases) {
    (void)id;
    interval_ms = std::min(interval_ms, lease.preferred_interval_ms);
  }
  return interval_ms;
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
        .field("kind", targetKindLabel(subscription.kind))
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

void SubscriptionLeaseManager::refreshDataInterval(const Subscription & subscription)
{
  if (isVideo(subscription) || subscription.leases.empty()) {
    return;
  }

  requireDataPublisher(subscription).setIntervalMs(appliedIntervalMs(subscription.leases));
}

void SubscriptionLeaseManager::pruneSubscriptionLeases(Clock::time_point reference_time)
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
      refreshDataInterval(subscription);
      ++it;
      continue;
    }

    LogEvent(kLogger, "subscription_pruned")
      .field("resource", subscription.name)
      .field("kind", targetKindLabel(subscription.kind))
      .field("reason", kLeaseExpiredReason)
      .info();
    // `subscription_pruned` already captures this lease-driven teardown boundary.
    destroy(subscription, false);
    it = subscriptions_.erase(it);
  }
}

void SubscriptionLeaseManager::destroy(Subscription & subscription, bool log_destroy)
{
  if (!isVideo(subscription)) {
    auto & data_publisher = requireDataPublisher(subscription);
    if (log_destroy) {
      LogEvent(kLogger, "subscription_destroyed")
        .field("resource", subscription.name)
        .field("kind", targetKindLabel(subscription.kind))
        .field("track_name", data_publisher.name())
        .info();
    }
    subscription.data_publisher.reset();
    return;
  }

  const auto & video_spec = requireVideoPublisher(subscription).spec();
  if (log_destroy) {
    LogEvent(kLogger, "subscription_destroyed")
      .field("resource", subscription.name)
      .field("kind", targetKindLabel(subscription.kind))
      .field("stream_key", video_spec.stream_key)
      .field("track_name", video_spec.track_name)
      .info();
  }
  subscription.video_publisher.reset();
}

void SubscriptionLeaseManager::shutdown()
{
  lease_gc_timer_.reset();

  if (is_shutdown_.exchange(true)) {
    return;
  }
  LogEvent(kLogger, "subscription_registry_shutdown_begin").field("subscription_count", subscriptions_.size()).info();
  session_leases_.clear();
  auto owned_subscriptions = std::move(subscriptions_);
  subscriptions_.clear();
  republish_requesters_.clear();

  for (auto & [key, subscription] : owned_subscriptions) {
    (void)key;
    destroy(subscription);
  }
}

}  // namespace livekit_ros2_bridge
