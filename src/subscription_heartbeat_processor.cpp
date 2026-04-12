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
constexpr auto kRequesterLeaseDuration = std::chrono::seconds(45);
constexpr auto kHeartbeatLogThrottlePeriod = std::chrono::seconds(5);

const char * subscriptionTargetKindString(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::ConfiguredSource:
      return "configured_source";
  }

  throw std::invalid_argument("heartbeat target kind is invalid");
}

nlohmann::json makeFlatTargetStatusJson(const SubscriptionTarget & target)
{
  const char * kind_str = subscriptionTargetKindString(target.kind);
  return {{"kind", kind_str}, {"name", target.name}};
}

void appendSubscriptionError(
  nlohmann::json & subscription_statuses,
  const SubscriptionTarget & target,
  const char * reason,
  const std::string & message)
{
  auto status_json = makeFlatTargetStatusJson(target);
  status_json["status"] = "error";
  status_json["error"] = {{"reason", reason}, {"message", message}};
  subscription_statuses.push_back(std::move(status_json));
}

const auto kHeartbeatProcessorLogger = rclcpp::get_logger("heartbeat_processor");
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

void SubscriptionHeartbeatProcessor::process(
  const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat)
{
  const auto requester_lease_expiry = std::chrono::steady_clock::now() + kRequesterLeaseDuration;
  const std::optional<std::string> resolved_requester_identity =
    resolveRequesterIdentity(requester_identity, heartbeat, requester_lease_expiry);
  if (!resolved_requester_identity.has_value()) {
    return;
  }

  nlohmann::json subscription_statuses = nlohmann::json::array();

  for (const auto & demand : heartbeat.subscriptions) {
    const auto & target = demand.target;

    if (target.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, target.name))
    {
      appendSubscriptionError(
        subscription_statuses, target, "forbidden", "ROS topic '" + target.name + "' not permitted.");
      continue;
    }

    try {
      auto subscription_status =
        subscription_registry_.renewSubscription(*resolved_requester_identity, demand, requester_lease_expiry);
      subscription_statuses.push_back(serializeSubscriptionStatus(subscription_status));
    } catch (const StreamUnavailableError & exc) {
      appendSubscriptionError(subscription_statuses, target, "unavailable", exc.what());
    } catch (const std::exception & exc) {
      appendSubscriptionError(subscription_statuses, target, "not_found", exc.what());
    }
  }

  // Refreshing the page reuses the requester identity before the old lease expires. We keep
  // that lease alive, but the rejoined participant still needs a fresh data-track publication
  // because the previous publication belonged to the disconnected participant_session.
  subscription_registry_.republishDataTracksForRequester(*resolved_requester_identity);
  publishSubscriptionStatus(
    *resolved_requester_identity, heartbeat.session_id, requester_lease_expiry, subscription_statuses);
}

void SubscriptionHeartbeatProcessor::pruneExpiredClientSessionLeases()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = client_session_leases_.begin(); it != client_session_leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    LogEvent(kHeartbeatProcessorLogger, "heartbeat_client_session_expired")
      .field("session_id", it->first)
      .field("requester_identity", it->second.requester_identity)
      .info();
    it = client_session_leases_.erase(it);
  }
}

std::optional<std::string> SubscriptionHeartbeatProcessor::resolveRequesterIdentity(
  const std::string & requester_identity,
  const SubscriptionHeartbeat & heartbeat,
  std::chrono::steady_clock::time_point requester_lease_expiry)
{
  if (!requester_identity.empty()) {
    if (heartbeat.session_id.has_value()) {
      if (!renewClientSessionLease(*heartbeat.session_id, requester_identity, requester_lease_expiry)) {
        return std::nullopt;
      }
    }
    return requester_identity;
  }

  if (!heartbeat.session_id.has_value()) {
    LogEvent(kHeartbeatProcessorLogger, "heartbeat_dropped")
      .field("reason", "anonymous_requester_without_client_session")
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
    return std::nullopt;
  }

  const auto lease_it = client_session_leases_.find(*heartbeat.session_id);
  if (lease_it == client_session_leases_.end()) {
    LogEvent(kHeartbeatProcessorLogger, "heartbeat_dropped")
      .field("reason", "unknown_session_id")
      .field("session_id", *heartbeat.session_id)
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
    return std::nullopt;
  }

  // LiveKit should normally attach the requester identity to user-data packets. When it does not,
  // we treat a known wire session_id as proof that this heartbeat belongs to the same previously
  // authenticated browser tab and continue renewing that client session lease instead of dropping
  // it.
  lease_it->second.expiry = requester_lease_expiry;
  LogEvent(kHeartbeatProcessorLogger, "heartbeat_client_session_fallback")
    .field("session_id", *heartbeat.session_id)
    .field("requester_identity", lease_it->second.requester_identity)
    .field("reason", "anonymous_requester")
    .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
  return lease_it->second.requester_identity;
}

bool SubscriptionHeartbeatProcessor::renewClientSessionLease(
  const std::string & client_session_id,
  const std::string & requester_identity,
  std::chrono::steady_clock::time_point requester_lease_expiry)
{
  auto it = client_session_leases_.find(client_session_id);
  if (it == client_session_leases_.end()) {
    client_session_leases_.emplace(client_session_id, ClientSessionLease{requester_identity, requester_lease_expiry});
    LogEvent(kHeartbeatProcessorLogger, "heartbeat_client_session_bound")
      .field("session_id", client_session_id)
      .field("requester_identity", requester_identity)
      .info();
    return true;
  }

  if (it->second.requester_identity != requester_identity) {
    if (const std::size_t count = client_session_conflict_throttle_.recordAndTakePendingCount(); count > 0U) {
      LogEvent(kHeartbeatProcessorLogger, "heartbeat_client_session_conflict")
        .field("reason", "requester_identity_mismatch")
        .field("session_id", client_session_id)
        .field("requester_identity", requester_identity)
        .field("existing_requester_identity", it->second.requester_identity)
        .field("count", count)
        .warn();
    }
    return false;
  }

  it->second.expiry = requester_lease_expiry;
  return true;
}

void SubscriptionHeartbeatProcessor::publishSubscriptionStatus(
  const std::string & requester_identity,
  const std::optional<std::string> & client_session_id,
  std::chrono::steady_clock::time_point requester_lease_expiry,
  const nlohmann::json & subscription_statuses)
{
  if (subscription_statuses.empty()) {
    return;
  }

  nlohmann::json envelope = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kControlSubscriptionsStatus},
    // The wire contract keeps the broad `subscriptions` array name even though each object is a
    // reported `SubscriptionStatus`.
    {"subscriptions", subscription_statuses},
  };
  if (client_session_id.has_value()) {
    const auto lease_expires_in_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(requester_lease_expiry - std::chrono::steady_clock::now())
        .count();
    envelope["session_id"] = *client_session_id;
    envelope["lease_expires_in_ms"] = lease_expires_in_ms;
  }

  const std::string serialized = envelope.dump();
  OutgoingControlPacket packet;
  packet.payload = std::vector<std::uint8_t>(serialized.begin(), serialized.end());
  packet.recipient_identities = {requester_identity};
  packet.control_topic = protocol::kControlSubscriptionsStatus;

  try {
    room_connection_.publishControlPacket(packet);
  } catch (const std::exception & exc) {
    LogEvent(kHeartbeatProcessorLogger, "subscription_status_publish_failed")
      .field("requester_identity", requester_identity)
      .field("control_topic", packet.control_topic)
      .field("error", exc.what())
      .warnThrottle(*clock_, kHeartbeatLogThrottlePeriod);
  }
}

}  // namespace livekit_ros2_bridge
