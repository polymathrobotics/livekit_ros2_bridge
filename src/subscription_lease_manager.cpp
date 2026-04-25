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

#include "data_track_publisher.hpp"
#include "protocol/constants.hpp"
#include "protocol/subscriptions_json.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("subscription_lease_manager");
constexpr auto kPruneInterval = std::chrono::seconds(1);
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

std::string makeKey(SubscriptionTargetKind kind, const std::string & name)
{
  const auto label = targetKindLabel(kind);
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

  if (kind == SubscriptionTargetKind::OtherVideo) {
    return trim(name);
  }

  return name;
}

}  // namespace

SubscriptionLeaseManager::DataRuntime::DataRuntime(std::shared_ptr<DataTrackPublisher> publisher)
: publisher(std::move(publisher))
{
  if (this->publisher == nullptr) {
    throw std::logic_error("data subscription runtime requires a publisher");
  }
}

SubscriptionLeaseManager::VideoRuntime::VideoRuntime(std::shared_ptr<VideoTrackPublisher> publisher)
: publisher(std::move(publisher))
{
  if (this->publisher == nullptr) {
    throw std::logic_error("video subscription runtime requires a publisher");
  }
}

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
: node_interfaces_(std::move(parameters), std::move(topics), std::move(graph))
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
    heartbeat = protocol::subscriptions::parse(payload);
  } catch (const std::exception & exc) {
    LogEvent event(kLogger, "packet_rejected");
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
  if (heartbeat.session_id.has_value()) {
    auto [it, inserted] = session_leases_.try_emplace(*heartbeat.session_id, SessionLease{requester, expiry});
    if (!inserted && it->second.requester_identity != requester) {
      throw std::logic_error("session lease invariant violated: session_id must resolve to one requester");
    }

    it->second.expiry = expiry;
  }

  SubscriptionStatusReport report;
  report.session_id = heartbeat.session_id;
  report.lease_expiry = expiry;
  report.statuses.reserve(heartbeat.demands.size());

  for (const auto & demand : heartbeat.demands) {
    // Bridge-owned `other_video` sources are config entries; subscribe ACLs apply to ROS topics.
    if (demand.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, demand.name))
    {
      report.statuses.emplace_back(
        SubscriptionErrorStatus{
          demand.kind,
          demand.name,
          SubscriptionErrorReason::Forbidden,
          "ROS topic '" + demand.name + "' not permitted.",
        });
      continue;
    }

    try {
      report.statuses.emplace_back(ensure(requester, demand, expiry));
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

  // Page refreshes can leave the data track attached to the old LiveKit participant session.
  republishTracks(requester);

  // Session-only heartbeats skip status envelopes.
  if (report.statuses.empty()) {
    return;
  }

  const std::string body = protocol::subscriptions::serialize(report);
  const std::vector<std::uint8_t> payload(body.begin(), body.end());
  const std::vector<std::string> destinations{requester};

  try {
    room_connection_.publishData(payload, true, destinations, protocol::kStatusTopic);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", requester)
      .fieldOr("session_id", heartbeat.session_id.value_or(""), "<absent>")
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
  }
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
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
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
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
        .field("existing_requester_identity", it->second.requester_identity)
        .field("count", pending)
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

SubscriptionLeaseManager::ResolvedDemand SubscriptionLeaseManager::resolveDemand(
  const SubscriptionDemand & demand) const
{
  ResolvedDemand resolved;
  resolved.kind = demand.kind;
  const auto topics = node_interfaces_.get_node_topics_interface();
  resolved.name = resolveName(*topics, demand.kind, demand.name);
  resolved.key = makeKey(demand.kind, resolved.name);

  if (demand.kind == SubscriptionTargetKind::Topic) {
    const auto graph = node_interfaces_.get_node_graph_interface();
    resolved.interface_type = requireSingleInterfaceType(graph->get_topic_names_and_types(), resolved.name, "topic");
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

  const int interval_ms = std::max(demand.preferred_interval_ms.value_or(0), 0);
  const Lease lease{interval_ms, expiry};
  const auto topics = node_interfaces_.get_node_topics_interface();
  const auto key = makeKey(demand.kind, resolveName(*topics, demand.kind, demand.name));
  if (auto it = subscriptions_.find(key); it != subscriptions_.end()) {
    return renew(it->second, requester_identity, lease);
  }

  return create(resolveDemand(demand), requester_identity, lease);
}

SubscriptionStatus SubscriptionLeaseManager::renew(
  Subscription & subscription, const std::string & requester_identity, const Lease & lease)
{
  try {
    const bool had_requester = subscription.leases.find(requester_identity) != subscription.leases.end();
    subscription.leases[requester_identity] = lease;

    auto * runtime = std::get_if<DataRuntime>(&subscription.runtime);
    if (runtime == nullptr) {
      return status(subscription);
    }

    auto & publisher = *runtime->publisher;
    const bool was_published = publisher.isPublished();
    publisher.setIntervalMs(minimumIntervalMs(subscription.leases));

    if (!had_requester && was_published) {
      // Status can reach a new participant before LiveKit surfaces the existing data track.
      republish_requesters_.insert(requester_identity);
    }

    if (!was_published) {
      publisher.publish();
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
  std::map<std::string, Lease> leases;
  leases.emplace(requester_identity, lease);

  try {
    Runtime runtime = [&]() -> Runtime {
      if (demand.video_spec.has_value()) {
        return VideoRuntime{
          VideoTrackPublisher::create(node_interfaces_, room_connection_, *demand.video_spec, qos_config_)};
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
      return DataRuntime{std::move(publisher)};
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
      .field("kind", targetKindLabel(demand.kind))
      .field("delivery", deliveryKindLabel(result.delivery))
      .field("requester_identity", requester_identity)
      .info();

    return result;
  } catch (...) {
    LogEvent(kLogger, "subscription_create_failed")
      .field("resource", demand.name)
      .field("kind", targetKindLabel(demand.kind))
      .field("requester_identity", requester_identity)
      .fieldException("error", std::current_exception())
      .warn();
    throw;
  }
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
    const auto * runtime = std::get_if<DataRuntime>(&subscription.runtime);
    if (runtime == nullptr) {
      continue;
    }

    const auto & publisher = *runtime->publisher;
    if (!publisher.isPublished()) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    // One published data track with a live lease is enough for republishTracks() to sweep the rest.
    republish_requesters_.insert(requester_identity);

    return;
  }
}

void SubscriptionLeaseManager::republishTracks(const std::string & requester_identity)
{
  if (is_shutdown_.load()) {
    return;
  }
  if (republish_requesters_.erase(requester_identity) == 0U) {
    return;
  }

  for (auto & [key, subscription] : subscriptions_) {
    (void)key;
    auto * runtime = std::get_if<DataRuntime>(&subscription.runtime);
    if (runtime == nullptr) {
      continue;
    }

    auto & publisher = *runtime->publisher;
    if (!publisher.isPublished()) {
      continue;
    }
    if (subscription.leases.find(requester_identity) == subscription.leases.end()) {
      continue;
    }

    LogEvent(kLogger, "data_track_republish")
      .field("resource", subscription.name)
      .field("track_name", publisher.trackName())
      .field("requester_identity", requester_identity)
      .info();
    publisher.republish();
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

    void operator()(const DataRuntime & runtime) const
    {
      const auto & publisher = *runtime.publisher;
      status.delivery = SubscriptionDeliveryKind::Data;
      if (publisher.isPublished()) {
        status.track_name = publisher.trackName();
      }
      status.interval_ms = publisher.intervalMs();
    }

    void operator()(const VideoRuntime & runtime) const
    {
      const auto & video_spec = runtime.publisher->spec();
      status.delivery = SubscriptionDeliveryKind::Video;
      status.track_name = video_spec.track_name;
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
      republish_requesters_.erase(lease_it->first);
      lease_it = subscription.leases.erase(lease_it);
    }

    if (!removed_any) {
      ++it;
      continue;
    }

    if (!subscription.leases.empty()) {
      if (const auto * runtime = std::get_if<DataRuntime>(&subscription.runtime); runtime != nullptr) {
        runtime->publisher->setIntervalMs(minimumIntervalMs(subscription.leases));
      }
      ++it;
      continue;
    }

    LogEvent(kLogger, "subscription_pruned")
      .field("resource", subscription.name)
      .field("kind", targetKindLabel(subscription.kind))
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
  republish_requesters_.clear();

  owned_subscriptions.clear();
}

}  // namespace livekit_ros2_bridge
