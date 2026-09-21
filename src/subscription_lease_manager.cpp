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
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "audio/stream_spec.hpp"
#include "audio/track_publisher.hpp"
#include "data_track_publisher.hpp"
#include "protocol/constants.hpp"
#include "protocol/subscriptions_json.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "ros_interfaces/graph_lookup.hpp"
#include "utils/log_event.hpp"
#include "utils/trim.hpp"
#include "video/stream_spec.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("subscription_lease_manager");
constexpr auto kPruneInterval = std::chrono::seconds(1);
constexpr const char * kLeaseExpiredReason = "lease_expired";

const video::StreamConfig & defaultStreamConfig()
{
  static const video::StreamConfig kDefaultConfig = video::makeDefaultConfig();
  return kDefaultConfig;
}

const audio::StreamConfig & defaultAudioStreamConfig()
{
  static const audio::StreamConfig kDefaultConfig = audio::makeDefaultConfig();
  return kDefaultConfig;
}

const char * targetKindKeyPrefix(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::ExternalVideo:
      return "external_video";
    case SubscriptionTargetKind::ExternalAudio:
      return "external_audio";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

std::string makeKey(SubscriptionTargetKind kind, const std::string & name)
{
  const auto label = targetKindKeyPrefix(kind);
  std::string key;
  key.reserve(std::char_traits<char>::length(label) + 1U + name.size());
  key.append(label);
  key.push_back(':');
  key.append(name);
  return key;
}

std::string resolveName(
  const rclcpp::node_interfaces::NodeTopicsInterface & topics, SubscriptionTargetKind kind, const std::string & name)
{
  if (kind == SubscriptionTargetKind::Topic) {
    return topics.resolve_topic_name(trim(name));
  }

  if (kind == SubscriptionTargetKind::ExternalVideo || kind == SubscriptionTargetKind::ExternalAudio) {
    return trim(name);
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
  const video::StreamConfig * video_stream_config,
  const audio::StreamConfig * audio_stream_config,
  Clock::duration heartbeat_lease_duration)
: node_interfaces_(std::move(parameters), std::move(topics), std::move(graph))
, clock_(std::move(clock))
, room_connection_(room_connection)
, access_policy_(std::move(access_policy))
, qos_config_(qos_config)
, video_stream_config_(video_stream_config)
, audio_stream_config_(audio_stream_config)
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
    heartbeat = protocol::subscriptions::parse(payload);
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "livekit_packet_rejected");
    event.field("reason", "invalid_heartbeat").fieldOr("requester_identity", requester_identity);
    if (const auto * validation = dynamic_cast<const protocol::ValidationError *>(&exc); validation != nullptr) {
      event.field("request_field", validation->field());
    }
    event.field("error", exc.what()).warnThrottle(*clock_, kLogThrottle);
    return;
  }

  handleHeartbeat(requester_identity, *heartbeat);
}

void SubscriptionLeaseManager::startPruneTimer(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers,
  ExecutorSubmitter submit_to_executor)
{
  prune_timer_ = rclcpp::create_wall_timer(
    kPruneInterval,
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
  const auto identity = resolveIdentity(requester_identity, heartbeat.session_id);
  if (!identity.has_value()) {
    return;
  }
  const auto & requester = *identity;

  const auto expiry = Clock::now() + heartbeat_lease_duration_;
  renewSessionLease(requester, heartbeat.session_id, expiry);

  const auto report = createStatusReport(heartbeat, requester, expiry);

  publishStatusReport(requester, heartbeat.session_id, report);
}

std::optional<std::string> SubscriptionLeaseManager::resolveIdentity(
  const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  if (requester_identity.empty()) {
    // A live client-session lease can authenticate anonymous heartbeats when LiveKit omits identity.
    const auto it = session_id.has_value() ? session_leases_.find(*session_id) : session_leases_.end();
    if (it == session_leases_.end()) {
      LogEvent(kLogger, "heartbeat_dropped")
        .field("reason", "anonymous_requester_without_resolvable_client_session")
        .fieldOr("session_id", session_id, "<absent>")
        .warnThrottle(*clock_, kLogThrottle);

      return std::nullopt;
    }

    return it->second.requester_identity;
  }

  if (!session_id.has_value()) {
    return requester_identity;
  }

  const auto it = session_leases_.find(*session_id);
  if (it != session_leases_.end() && it->second.requester_identity != requester_identity) {
    if (const std::size_t pending = conflict_throttle_.record(); pending > 0U) {
      LogEvent(kLogger, "heartbeat_client_session_conflict")
        .field("requester_identity", requester_identity)
        .fieldOr("session_id", session_id, "<absent>")
        .field("existing_requester_identity", it->second.requester_identity)
        .field("count", pending)
        .warn();
    }

    return std::nullopt;
  }

  return requester_identity;
}

void SubscriptionLeaseManager::renewSessionLease(
  const std::string & requester_identity, const std::optional<std::string> & session_id, Clock::time_point expiry)
{
  if (!session_id.has_value()) {
    return;
  }

  auto [it, inserted] = session_leases_.try_emplace(*session_id, SessionLease{requester_identity, expiry});
  if (!inserted && it->second.requester_identity != requester_identity) {
    throw std::logic_error("session lease invariant violated: session_id must resolve to one requester");
  }

  it->second.expiry = expiry;
}

void SubscriptionLeaseManager::pruneSessionLeases(Clock::time_point now)
{
  for (auto it = session_leases_.begin(); it != session_leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    it = session_leases_.erase(it);
  }
}

const video::StreamConfig & SubscriptionLeaseManager::videoStreamConfig() const
{
  return video_stream_config_ == nullptr ? defaultStreamConfig() : *video_stream_config_;
}

const audio::StreamConfig & SubscriptionLeaseManager::audioStreamConfig() const
{
  return audio_stream_config_ == nullptr ? defaultAudioStreamConfig() : *audio_stream_config_;
}

video::StreamSpec SubscriptionLeaseManager::resolveVideoSpec(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return video::resolveRosTopicSpec(videoStreamConfig(), name, interface_type);
    case SubscriptionTargetKind::ExternalVideo:
      return video::resolveExternalSourceSpec(videoStreamConfig(), name);
    case SubscriptionTargetKind::ExternalAudio:
      throw std::invalid_argument("external audio is not a video stream request");
  }

  // All SubscriptionTargetKind enumerators are handled above; the switch cannot
  // fall through here, but GCC still requires a return path after the switch.
  throw std::logic_error("unreachable video stream request kind");
}

audio::StreamSpec SubscriptionLeaseManager::resolveAudioSpec(
  SubscriptionTargetKind kind, const std::string & name) const
{
  switch (kind) {
    case SubscriptionTargetKind::ExternalAudio:
      return audio::resolveExternalSourceSpec(audioStreamConfig(), name);
    case SubscriptionTargetKind::Topic:
    case SubscriptionTargetKind::ExternalVideo:
      throw std::invalid_argument("only external audio requests resolve to audio streams");
  }

  // All SubscriptionTargetKind enumerators are handled above; the switch cannot
  // fall through here, but GCC still requires a return path after the switch.
  throw std::logic_error("unreachable audio stream request kind");
}

SubscriptionLeaseManager::ResolvedDemand SubscriptionLeaseManager::resolveDemand(
  const SubscriptionDemand & demand) const
{
  ResolvedDemand resolved;
  resolved.kind = demand.kind;
  resolved.preferred_interval_ms = std::max(demand.preferred_interval_ms.value_or(0), 0);
  const auto topics = node_interfaces_.get_node_topics_interface();
  resolved.name = resolveName(*topics, demand.kind, demand.name);
  resolved.key = makeKey(demand.kind, resolved.name);
  return resolved;
}

void SubscriptionLeaseManager::resolveDemandDelivery(ResolvedDemand & demand) const
{
  if (demand.kind == SubscriptionTargetKind::Topic) {
    const auto graph = node_interfaces_.get_node_graph_interface();
    demand.interface_type = ros_interfaces::requireSingleType(graph->get_topic_names_and_types(), demand.name, "topic");
    if (!video::classifyRosIngestMode(demand.interface_type).has_value()) {
      return;
    }
  }

  if (demand.kind == SubscriptionTargetKind::ExternalAudio) {
    demand.audio_spec = resolveAudioSpec(demand.kind, demand.name);
    return;
  }

  demand.video_spec = resolveVideoSpec(demand.kind, demand.name, demand.interface_type);
}

SubscriptionStatusReport SubscriptionLeaseManager::createStatusReport(
  const SubscriptionHeartbeat & heartbeat, const std::string & requester_identity, Clock::time_point expiry)
{
  SubscriptionStatusReport report;
  report.session_id = heartbeat.session_id;
  report.lease_expiry = expiry;
  report.statuses.reserve(heartbeat.demands.size());

  for (const auto & demand : heartbeat.demands) {
    appendDemandStatus(report, requester_identity, demand, expiry);
  }

  return report;
}

void SubscriptionLeaseManager::appendDemandStatus(
  SubscriptionStatusReport & report,
  const std::string & requester_identity,
  const SubscriptionDemand & demand,
  Clock::time_point expiry)
{
  // Bridge-owned `external_video` sources are config entries; subscribe ACLs apply to ROS topics.
  if (demand.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, demand.name)) {
    report.statuses.emplace_back(
      SubscriptionErrorStatus{
        demand.kind,
        demand.name,
        SubscriptionErrorReason::Forbidden,
        "ROS topic '" + demand.name + "' not permitted.",
      });
    return;
  }

  try {
    if (is_shutdown_.load()) {
      throw std::runtime_error("Subscription registry is shut down.");
    }
    const auto resolved = resolveDemand(demand);
    auto subscription_status = ensure(requester_identity, resolved, expiry);
    report.statuses.emplace_back(std::move(subscription_status));
  } catch (const std::exception & exc) {
    report.statuses.emplace_back(
      SubscriptionErrorStatus{
        demand.kind,
        demand.name,
        SubscriptionErrorReason::NotFound,
        exc.what(),
      });
  }
}

EchoOnceResult SubscriptionLeaseManager::dispatchEchoOnce(
  SubscriptionTargetKind kind, const std::string & name, const std::string & requester_identity)
{
  // Key off the canonically resolved name, exactly as subscriptions are keyed, so a request finds
  // the publisher another client's subscription created and cached.
  const auto topics = node_interfaces_.get_node_topics_interface();
  const auto it = subscriptions_.find(makeKey(kind, resolveName(*topics, kind, name)));
  if (it == subscriptions_.end()) {
    return EchoOnceResult::None;
  }

  // Only data deliveries carry a cache; video and other non-data runtimes never do.
  const auto * data_publisher_ptr = std::get_if<DataPublisher>(&it->second.runtime);
  if (data_publisher_ptr == nullptr) {
    return EchoOnceResult::None;
  }

  const auto snapshot = (*data_publisher_ptr)->cachedMessage();
  if (!snapshot.has_value()) {
    return EchoOnceResult::None;
  }

  try {
    // The send is non-blocking (fire-and-forget on a detached thread), so a later transfer failure
    // is logged inside the sender, not surfaced here. We report Sent the moment the cached bytes are
    // handed off. `name` carries the resolved ROS topic so the client's one handler can route it.
    room_connection_.sendByteStream(
      protocol::kEchoOnceTopic,
      /*name=*/snapshot->name,
      protocol::kCdrContentType,
      snapshot->cdr,
      requester_identity);
  } catch (const std::exception & exc) {
    // The dispatch failed synchronously (e.g. the room dropped between lookup and send). Report None
    // so the client's retry loop tries again rather than assuming a delivery that never left.
    LogEvent(kLogger, "echo_once_send_failed")
      .field("resource", snapshot->name)
      .field("requester_identity", requester_identity)
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
    return EchoOnceResult::None;
  }

  return EchoOnceResult::Sent;
}

void SubscriptionLeaseManager::publishStatusReport(
  const std::string & requester_identity,
  const std::optional<std::string> & session_id,
  const SubscriptionStatusReport & report)
{
  // Session-only heartbeats skip status envelopes.
  if (report.statuses.empty()) {
    return;
  }

  const std::string body = protocol::subscriptions::serialize(report);
  const std::vector<std::uint8_t> payload(body.begin(), body.end());
  const std::vector<std::string> destinations{requester_identity};

  try {
    room_connection_.publishData(payload, true, destinations, protocol::kStatusTopic);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", requester_identity)
      .fieldOr("session_id", session_id, "<absent>")
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
  }
}

SubscriptionStatus SubscriptionLeaseManager::ensure(
  const std::string & requester_identity, const ResolvedDemand & demand, Clock::time_point expiry)
{
  if (is_shutdown_.load()) {
    throw std::runtime_error("Subscription registry is shut down.");
  }

  const Lease lease{demand.preferred_interval_ms, expiry};
  if (auto it = subscriptions_.find(demand.key); it != subscriptions_.end()) {
    return renew(it->second, requester_identity, lease);
  }

  auto create_demand = demand;
  resolveDemandDelivery(create_demand);
  return create(create_demand, requester_identity, lease);
}

SubscriptionStatus SubscriptionLeaseManager::renew(
  Subscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    subscription.leases[requester_identity] = lease;

    auto * runtime = std::get_if<DataPublisher>(&subscription.runtime);
    if (runtime == nullptr) {
      return status(subscription);
    }

    auto & publisher = **runtime;
    const bool was_published = publisher.isPublished();
    publisher.setIntervalMs(minimumIntervalMs(subscription.leases));

    if (!was_published) {
      publisher.publish();
    }
  } catch (...) {
    LogEvent(kLogger, "subscription_renew_failed")
      .field("resource", subscription.name)
      .fieldEnum("kind", subscription.kind)
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
  std::map<std::string, Lease> leases;
  leases.emplace(requester_identity, lease);

  try {
    Runtime runtime = [&]() -> Runtime {
      if (demand.audio_spec.has_value()) {
        return Runtime{audio::TrackPublisher::create(room_connection_, *demand.audio_spec)};
      }
      if (demand.video_spec.has_value()) {
        return Runtime{
          video::TrackPublisher::create(node_interfaces_, room_connection_, *demand.video_spec, qos_config_)};
      }

      auto publisher = std::make_shared<DataTrackPublisher>(
        demand.name,
        demand.interface_type,
        node_interfaces_.get_node_topics_interface(),
        node_interfaces_.get_node_graph_interface(),
        clock_,
        room_connection_,
        qos_config_);
      publisher->setIntervalMs(minimumIntervalMs(leases));
      publisher->publish();
      return Runtime{std::move(publisher)};
    }();

    Subscription subscription{
      demand.kind,
      demand.name,
      demand.interface_type,
      std::move(leases),
      std::move(runtime),
    };
    auto it = subscriptions_.emplace(demand.key, std::move(subscription)).first;

    SubscriptionStatus result = status(it->second);
    LogEvent(kLogger, "subscription_created")
      .field("resource", it->second.name)
      .fieldEnum("kind", demand.kind)
      .fieldEnum("delivery", result.delivery)
      .field("requester_identity", requester_identity)
      .info();

    return result;
  } catch (...) {
    LogEvent(kLogger, "subscription_create_failed")
      .field("resource", demand.name)
      .fieldEnum("kind", demand.kind)
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();
    throw;
  }
}

void SubscriptionLeaseManager::pruneExpiredLeases()
{
  if (is_shutdown_.load()) {
    return;
  }

  const auto now = Clock::now();
  pruneSessionLeases(now);
  pruneLeases(now);
}

SubscriptionStatus SubscriptionLeaseManager::status(const Subscription & subscription) const
{
  SubscriptionStatus status;
  status.kind = subscription.kind;
  status.name = subscription.name;
  status.interface_type = subscription.interface_type;

  struct StatusVisitor
  {
    SubscriptionStatus & status;

    void operator()(const DataPublisher & publisher_ptr) const
    {
      const auto & publisher = *publisher_ptr;
      status.delivery = SubscriptionDeliveryKind::Data;
      if (publisher.isPublished()) {
        status.track_name = publisher.trackName();
        // Durability is only known once the publication (and its resolved subscription QoS) exists;
        // before then leave qos unset so the wire omits it rather than asserting "volatile".
        status.qos = publisher.qos();
      }
      status.interval_ms = publisher.intervalMs();
    }

    void operator()(const VideoPublisher & publisher) const
    {
      const auto & video_spec = publisher->spec();
      status.delivery = SubscriptionDeliveryKind::Video;
      status.track_name = video_spec.track_name;
    }

    void operator()(const AudioPublisher & publisher) const
    {
      const auto & audio_spec = publisher->spec();
      status.delivery = SubscriptionDeliveryKind::Audio;
      status.track_name = audio_spec.track_name;
    }
  };

  std::visit(StatusVisitor{status}, subscription.runtime);
  return status;
}

int SubscriptionLeaseManager::minimumIntervalMs(const std::map<std::string, Lease> & leases)
{
  int interval_ms = leases.begin()->second.preferred_interval_ms;
  for (const auto & [id, lease] : leases) {
    (void)id;
    interval_ms = std::min(interval_ms, lease.preferred_interval_ms);
  }
  return interval_ms;
}

void SubscriptionLeaseManager::pruneLeases(Clock::time_point now)
{
  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    auto & subscription = it->second;
    bool removed_any = false;
    for (auto lease_it = subscription.leases.begin(); lease_it != subscription.leases.end();) {
      if (now < lease_it->second.expiry) {
        ++lease_it;
        continue;
      }

      removed_any = true;
      lease_it = subscription.leases.erase(lease_it);
    }

    if (!removed_any) {
      ++it;
      continue;
    }

    if (!subscription.leases.empty()) {
      if (const auto * publisher = std::get_if<DataPublisher>(&subscription.runtime); publisher != nullptr) {
        (*publisher)->setIntervalMs(minimumIntervalMs(subscription.leases));
      }
      ++it;
      continue;
    }

    LogEvent(kLogger, "subscription_pruned")
      .field("resource", subscription.name)
      .fieldEnum("kind", subscription.kind)
      .field("reason", kLeaseExpiredReason)
      .info();
    it = subscriptions_.erase(it);
  }
}

void SubscriptionLeaseManager::shutdown()
{
  prune_timer_.reset();

  if (is_shutdown_.exchange(true)) {
    return;
  }
  if (!subscriptions_.empty()) {
    LogEvent(kLogger, "subscription_registry_shutdown_begin").field("subscription_count", subscriptions_.size()).info();
  }
  session_leases_.clear();
  auto owned_subscriptions = std::move(subscriptions_);
  subscriptions_.clear();

  owned_subscriptions.clear();
}

}  // namespace livekit_ros2_bridge
