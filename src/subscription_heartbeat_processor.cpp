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

#include "subscription_heartbeat_processor.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "nlohmann/json.hpp"
#include "payloads/stream_control_payloads.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "subscription_registry.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{
constexpr auto kHeartbeatLeaseDuration = std::chrono::seconds(45);
constexpr auto kHeartbeatLogThrottlePeriod = std::chrono::seconds(5);

LogEvent & appendRequesterSessionFields(
  LogEvent & event, const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  return event.field("requester_identity", requester_identity)
    .fieldOr("session_id", session_id.value_or(""), "<absent>");
}

void appendSubscriptionErrorStatus(
  nlohmann::json & statuses, const SubscriptionTarget & target, const char * reason, const std::string & message)
{
  statuses.push_back({
    {"kind", subscriptionTargetKindString(target.kind)},
    {"name", target.name},
    {"status", "error"},
    {"error", {{"reason", reason}, {"message", message}}},
  });
}

const auto kLogger = rclcpp::get_logger("heartbeat_processor");
}  // namespace

SubscriptionHeartbeatProcessor::SubscriptionHeartbeatProcessor(
  SubscriptionRegistry & subscription_registry,
  RoomConnection & room_connection,
  AccessPolicy access_policy,
  rclcpp::Clock::SharedPtr clock)
: subscription_registry_(subscription_registry)
, room_connection_(room_connection)
, access_policy_(std::move(access_policy))
, clock_(std::move(clock))
{}

std::optional<SubscriptionHeartbeatProcessor::ResolvedHeartbeatLease>
SubscriptionHeartbeatProcessor::resolveHeartbeatLease(
  const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  const auto lease_expiry = std::chrono::steady_clock::now() + kHeartbeatLeaseDuration;
  if (requester_identity.empty()) {
    auto resolved_identity = resolveAnonymousIdentity(session_id, lease_expiry);
    if (!resolved_identity.has_value()) {
      return std::nullopt;
    }

    return ResolvedHeartbeatLease{std::move(*resolved_identity), session_id, lease_expiry};
  }

  if (!session_id.has_value()) {
    return ResolvedHeartbeatLease{requester_identity, session_id, lease_expiry};
  }

  auto [it, inserted] = session_leases_.try_emplace(*session_id, SessionLease{requester_identity, lease_expiry});
  if (inserted) {
    return ResolvedHeartbeatLease{requester_identity, session_id, lease_expiry};
  }

  if (it->second.requester_identity != requester_identity) {
    if (const std::size_t count = session_conflict_throttle_.recordAndTakePendingCount(); count > 0U) {
      auto event = LogEvent(kLogger, "heartbeat_client_session_conflict");
      appendRequesterSessionFields(event, requester_identity, session_id)
        .field("existing_requester_identity", it->second.requester_identity)
        .field("count", count)
        .warn();
    }
    return std::nullopt;
  }

  it->second.expiry = lease_expiry;
  return ResolvedHeartbeatLease{requester_identity, session_id, lease_expiry};
}

std::optional<std::string> SubscriptionHeartbeatProcessor::resolveAnonymousIdentity(
  const std::optional<std::string> & session_id, std::chrono::steady_clock::time_point lease_expiry)
{
  if (!session_id.has_value()) {
    LogEvent(kLogger, "heartbeat_dropped")
      .field("reason", "anonymous_requester_without_client_session")
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
    return std::nullopt;
  }

  const auto it = session_leases_.find(*session_id);
  if (it == session_leases_.end()) {
    LogEvent(kLogger, "heartbeat_dropped")
      .field("reason", "unknown_session_id")
      .field("session_id", *session_id)
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
    return std::nullopt;
  }

  // LiveKit should normally attach the requester identity to user-data packets. If it does not, a
  // known wire session_id is enough to treat the heartbeat as belonging to the same authenticated
  // browser tab and renew that client-session lease instead of dropping it.
  it->second.expiry = lease_expiry;

  auto event = LogEvent(kLogger, "heartbeat_client_session_fallback");
  appendRequesterSessionFields(event, it->second.requester_identity, session_id)
    .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
  return it->second.requester_identity;
}

nlohmann::json SubscriptionHeartbeatProcessor::renewSubscriptionStatuses(
  const ResolvedHeartbeatLease & lease, const std::vector<SubscriptionDemand> & subscriptions)
{
  nlohmann::json statuses = nlohmann::json::array();

  for (const auto & demand : subscriptions) {
    const auto & target = demand.target;

    // `configured_source` targets name bridge-owned config entries rather than ROS graph
    // resources, so subscribe ACLs apply only to true ROS topic subscriptions here.
    if (target.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, target.name))
    {
      appendSubscriptionErrorStatus(statuses, target, "forbidden", "ROS topic '" + target.name + "' not permitted.");
      continue;
    }

    try {
      auto status = subscription_registry_.renewSubscription(lease.requester_identity, demand, lease.expiry);
      statuses.push_back(stream_control_payloads::serializeSubscriptionStatus(status));
    } catch (const StreamUnavailableError & exc) {
      appendSubscriptionErrorStatus(statuses, target, "unavailable", exc.what());
    } catch (const std::exception & exc) {
      appendSubscriptionErrorStatus(statuses, target, "not_found", exc.what());
    }
  }

  return statuses;
}

void SubscriptionHeartbeatProcessor::pruneExpiredLeases()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = session_leases_.begin(); it != session_leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    it = session_leases_.erase(it);
  }
}

void SubscriptionHeartbeatProcessor::process(
  const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat)
{
  const auto lease = resolveHeartbeatLease(requester_identity, heartbeat.session_id);
  if (!lease.has_value()) {
    return;
  }

  const auto statuses = renewSubscriptionStatuses(*lease, heartbeat.subscriptions);

  // A page refresh can reuse the requester identity before the old lease expires, but the
  // rejoined participant still needs a fresh data-track publication because the previous one
  // belonged to the disconnected participant_session.
  subscription_registry_.republishDataTracks(lease->requester_identity);
  publishStatuses(*lease, statuses);
}

void SubscriptionHeartbeatProcessor::publishStatuses(
  const ResolvedHeartbeatLease & lease, const nlohmann::json & statuses)
{
  // A heartbeat may exist only to bind or renew the client-session lease. In that case the wire
  // contract does not send an empty status envelope back.
  if (statuses.empty()) {
    return;
  }

  nlohmann::json envelope = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kControlSubscriptionsStatus},
    // The wire contract keeps the broad `subscriptions` array name even though each object is a
    // reported `SubscriptionStatus`.
    {"subscriptions", statuses},
  };
  if (lease.session_id.has_value()) {
    const auto lease_expires_in_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(lease.expiry - std::chrono::steady_clock::now()).count();
    envelope["session_id"] = *lease.session_id;
    envelope["lease_expires_in_ms"] = lease_expires_in_ms;
  }

  const std::string body = envelope.dump();
  OutgoingControlPacket packet;
  packet.payload = std::vector<std::uint8_t>(body.begin(), body.end());
  packet.recipient_identities = {lease.requester_identity};
  packet.control_topic = protocol::kControlSubscriptionsStatus;

  try {
    room_connection_.publishControlPacket(packet);
  } catch (const std::exception & exc) {
    auto event = LogEvent(kLogger, "subscription_status_publish_failed");
    appendRequesterSessionFields(event, lease.requester_identity, lease.session_id)
      .field("error", exc.what())
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
  }
}

}  // namespace livekit_ros2_bridge
