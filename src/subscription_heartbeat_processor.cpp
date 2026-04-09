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
#include "room_session.hpp"
#include "subscription_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{
constexpr auto kLeaseExpiry = std::chrono::seconds(45);

const char * subscriptionTargetKindString(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::External:
      return "external";
  }

  throw std::invalid_argument("heartbeat target kind is invalid");
}

nlohmann::json makeFlatTargetEntry(const SubscriptionTarget & target)
{
  const char * kind_str = subscriptionTargetKindString(target.kind);
  const char * name_key = target.kind == SubscriptionTargetKind::Topic ? "topic" : "external";
  return {{"kind", kind_str}, {name_key, target.name}};
}

void appendStreamError(
  nlohmann::json & streams, const SubscriptionTarget & target, const char * reason, const std::string & message)
{
  auto error_entry = makeFlatTargetEntry(target);
  error_entry["status"] = "error";
  error_entry["error"] = {{"reason", reason}, {"message", message}};
  streams.push_back(std::move(error_entry));
}

const auto kHeartbeatProcessorLogger = rclcpp::get_logger("heartbeat_processor");
}  // namespace

SubscriptionHeartbeatProcessor::SubscriptionHeartbeatProcessor(
  SubscriptionRegistry & subscription_registry,
  RoomSession & session,
  AccessPolicy access_policy,
  rclcpp::Clock::SharedPtr clock)
: subscription_registry_(subscription_registry)
, room_session_(session)
, access_policy_(std::move(access_policy))
, clock_(std::move(clock))
{}

void SubscriptionHeartbeatProcessor::process(
  const std::string & requester_identity, const SubscriptionHeartbeat & update)
{
  const auto expiry = std::chrono::steady_clock::now() + kLeaseExpiry;
  const std::optional<std::string> resolved_requester_identity =
    resolveRequesterIdentity(requester_identity, update, expiry);
  if (!resolved_requester_identity.has_value()) {
    return;
  }

  nlohmann::json streams = nlohmann::json::array();

  for (const auto & entry : update.subscriptions) {
    const auto & target = entry.target;

    if (target.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, target.name))
    {
      appendStreamError(streams, target, "forbidden", "ROS topic '" + target.name + "' not permitted.");
      continue;
    }

    try {
      auto stream_status = subscription_registry_.renewSubscription(*resolved_requester_identity, entry, expiry);
      streams.push_back(serializeStreamStatus(stream_status));
    } catch (const StreamUnavailableError & exc) {
      appendStreamError(streams, target, "unavailable", exc.what());
    } catch (const std::exception & exc) {
      appendStreamError(streams, target, "not_found", exc.what());
    }
  }

  // Refreshing the page reuses the requester identity before the old lease expires. We keep
  // that lease alive, but the rejoined participant still needs a fresh data-track publication
  // because the previous publication belonged to the disconnected participant session.
  subscription_registry_.replayCdrTracksForRequester(*resolved_requester_identity);
  publishSubscriptionStatus(*resolved_requester_identity, update.session_id, expiry, streams);
}

void SubscriptionHeartbeatProcessor::sweepExpiredSessionLeases()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = session_leases_.begin(); it != session_leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    RCLCPP_INFO(
      kHeartbeatProcessorLogger,
      "event=heartbeat_session_expired session_id=%s requester_identity=%s",
      it->first.c_str(),
      it->second.requester_identity.c_str());
    it = session_leases_.erase(it);
  }
}

std::optional<std::string> SubscriptionHeartbeatProcessor::resolveRequesterIdentity(
  const std::string & requester_identity,
  const SubscriptionHeartbeat & update,
  std::chrono::steady_clock::time_point expiry)
{
  if (!requester_identity.empty()) {
    if (update.session_id.has_value()) {
      if (!bindSessionId(*update.session_id, requester_identity, expiry)) {
        return std::nullopt;
      }
    }
    return requester_identity;
  }

  if (!update.session_id.has_value()) {
    RCLCPP_WARN_THROTTLE(
      kHeartbeatProcessorLogger, *clock_, 5000, "event=heartbeat_dropped reason=anonymous_requester_without_session");
    return std::nullopt;
  }

  const auto lease_it = session_leases_.find(*update.session_id);
  if (lease_it == session_leases_.end()) {
    RCLCPP_WARN_THROTTLE(
      kHeartbeatProcessorLogger,
      *clock_,
      5000,
      "event=heartbeat_dropped reason=unknown_session_id session_id=%s",
      update.session_id->c_str());
    return std::nullopt;
  }

  // LiveKit should normally attach the requester identity to user-data packets. When it does not,
  // we treat a known session_id as proof that this heartbeat belongs to the same previously
  // authenticated browser tab and continue renewing that tab's leases instead of dropping them.
  lease_it->second.expiry = expiry;
  RCLCPP_WARN_THROTTLE(
    kHeartbeatProcessorLogger,
    *clock_,
    5000,
    "event=heartbeat_session_fallback session_id=%s requester_identity=%s reason=anonymous_requester",
    update.session_id->c_str(),
    lease_it->second.requester_identity.c_str());
  return lease_it->second.requester_identity;
}

bool SubscriptionHeartbeatProcessor::bindSessionId(
  const std::string & session_id, const std::string & requester_identity, std::chrono::steady_clock::time_point expiry)
{
  auto it = session_leases_.find(session_id);
  if (it == session_leases_.end()) {
    session_leases_.emplace(session_id, SessionLease{requester_identity, expiry});
    RCLCPP_INFO(
      kHeartbeatProcessorLogger,
      "event=heartbeat_session_bound session_id=%s requester_identity=%s",
      session_id.c_str(),
      requester_identity.c_str());
    return true;
  }

  if (it->second.requester_identity != requester_identity) {
    RCLCPP_WARN(
      kHeartbeatProcessorLogger,
      "event=heartbeat_session_conflict session_id=%s requester_identity=%s existing_requester_identity=%s",
      session_id.c_str(),
      requester_identity.c_str(),
      it->second.requester_identity.c_str());
    return false;
  }

  it->second.expiry = expiry;
  return true;
}

void SubscriptionHeartbeatProcessor::publishSubscriptionStatus(
  const std::string & requester_identity,
  const std::optional<std::string> & session_id,
  std::chrono::steady_clock::time_point expiry,
  const nlohmann::json & streams)
{
  if (streams.empty()) {
    return;
  }

  nlohmann::json envelope = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kControlSubscriptionsStatus},
    {"streams", streams},
  };
  if (session_id.has_value()) {
    envelope["session_id"] = *session_id;
    envelope["lease_expires_in_ms"] =
      std::chrono::duration_cast<std::chrono::milliseconds>(expiry - std::chrono::steady_clock::now()).count();
  }

  const std::string serialized = envelope.dump();
  OutgoingControlPacket packet;
  packet.payload = std::vector<std::uint8_t>(serialized.begin(), serialized.end());
  packet.recipient_identities = {requester_identity};
  packet.control_topic = protocol::kControlSubscriptionsStatus;

  try {
    room_session_.publishControlPacket(packet);
  } catch (const std::exception & exc) {
    RCLCPP_WARN_THROTTLE(
      kHeartbeatProcessorLogger,
      *clock_,
      5000,
      "Failed to send stream status to %s: %s",
      requester_identity.c_str(),
      exc.what());
  }
}

}  // namespace livekit_ros2_bridge
