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
#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "subscription_registry.hpp"
#include "utils/log_event.hpp"
#include "wire/subscriptions.hpp"

namespace livekit_ros2_bridge
{

namespace
{
constexpr auto kHeartbeatLeaseDuration = std::chrono::seconds(45);

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

void SubscriptionHeartbeatProcessor::process(
  const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat)
{
  const auto lease = resolveLease(requester_identity, heartbeat.session_id);
  if (!lease.has_value()) {
    return;
  }

  std::vector<SubscriptionReportedStatus> statuses;
  statuses.reserve(heartbeat.subscriptions.size());

  for (const auto & demand : heartbeat.subscriptions) {
    statuses.emplace_back(renewSubscription(*lease, demand));
  }

  // A page refresh can reuse the requester identity before the old lease expires, but the
  // rejoined participant still needs a fresh data-track publication because the previous one
  // belonged to the disconnected participant_session.
  subscription_registry_.republishDataTracks(lease->requester_identity);

  publishStatuses(*lease, statuses);
}

void SubscriptionHeartbeatProcessor::pruneExpiredLeases()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = leases_.begin(); it != leases_.end();) {
    if (now < it->second.expiry) {
      ++it;
      continue;
    }

    it = leases_.erase(it);
  }
}

std::optional<SubscriptionHeartbeatProcessor::ResolvedLease> SubscriptionHeartbeatProcessor::resolveLease(
  const std::string & requester_identity, const std::optional<std::string> & session_id)
{
  const auto expiry = std::chrono::steady_clock::now() + kHeartbeatLeaseDuration;
  if (requester_identity.empty()) {
    // LiveKit should normally attach the requester identity to user-data packets. If it does not, a
    // known wire session_id is enough to treat the heartbeat as belonging to the same authenticated
    // browser tab and renew that client-session lease instead of dropping it.
    const auto it = session_id.has_value() ? leases_.find(*session_id) : leases_.end();
    if (it == leases_.end()) {
      LogEvent(kLogger, "heartbeat_dropped")
        .field("reason", "anonymous_requester_without_resolvable_client_session")
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
        .warnThrottle(*clock_, kLogThrottle);
      return std::nullopt;
    }

    it->second.expiry = expiry;
    LogEvent(kLogger, "heartbeat_client_session_fallback")
      .field("requester_identity", it->second.requester_identity)
      .fieldOr("session_id", session_id.value_or(""), "<absent>")
      .warnThrottle(*clock_, kLogThrottle);
    return ResolvedLease{it->second.requester_identity, session_id, expiry};
  }

  if (!session_id.has_value()) {
    return ResolvedLease{requester_identity, session_id, expiry};
  }

  auto [it, inserted] = leases_.try_emplace(*session_id, SessionLease{requester_identity, expiry});
  auto & lease = it->second;
  if (!inserted && lease.requester_identity != requester_identity) {
    if (const std::size_t count = conflict_throttle_.recordAndTakePendingCount(); count > 0U) {
      LogEvent(kLogger, "heartbeat_client_session_conflict")
        .field("requester_identity", requester_identity)
        .fieldOr("session_id", session_id.value_or(""), "<absent>")
        .field("existing_requester_identity", lease.requester_identity)
        .field("count", count)
        .warn();
    }
    return std::nullopt;
  }

  lease.expiry = expiry;
  return ResolvedLease{requester_identity, session_id, expiry};
}

SubscriptionReportedStatus SubscriptionHeartbeatProcessor::renewSubscription(
  const ResolvedLease & lease, const SubscriptionDemand & demand)
{
  const auto & target = demand.target;

  // `configured_source` targets name bridge-owned config entries rather than ROS graph
  // resources, so subscribe ACLs apply only to true ROS topic subscriptions here.
  if (target.kind == SubscriptionTargetKind::Topic && !access_policy_.allows(AccessOperation::Subscribe, target.name)) {
    return SubscriptionErrorStatus{
      target,
      SubscriptionStatusErrorReason::kForbidden,
      "ROS topic '" + target.name + "' not permitted.",
    };
  }

  try {
    return subscription_registry_.renewSubscription(lease.requester_identity, demand, lease.expiry);
  } catch (const StreamUnavailableError & exc) {
    return SubscriptionErrorStatus{
      target,
      SubscriptionStatusErrorReason::kUnavailable,
      exc.what(),
    };
  } catch (const std::exception & exc) {
    return SubscriptionErrorStatus{
      target,
      SubscriptionStatusErrorReason::kNotFound,
      exc.what(),
    };
  }
}

void SubscriptionHeartbeatProcessor::publishStatuses(
  const ResolvedLease & lease, const std::vector<SubscriptionReportedStatus> & statuses)
{
  // A heartbeat may exist only to bind or renew the client-session lease. In that case the wire
  // contract does not send an empty status envelope back.
  if (statuses.empty()) {
    return;
  }

  std::optional<wire::subscriptions::SubscriptionStatusLease> lease_metadata;
  if (lease.session_id.has_value()) {
    lease_metadata = wire::subscriptions::SubscriptionStatusLease{*lease.session_id, lease.expiry};
  }

  const std::string body = wire::subscriptions::serializeSubscriptionStatuses(statuses, lease_metadata).dump();
  OutgoingPacket packet;
  packet.payload = std::vector<std::uint8_t>(body.begin(), body.end());
  packet.recipient_identities = {lease.requester_identity};
  packet.topic = wire::protocol::kControlSubscriptionsStatus;

  try {
    room_connection_.publishPacket(packet);
  } catch (const std::exception & exc) {
    LogEvent(kLogger, "subscription_status_publish_failed")
      .field("requester_identity", lease.requester_identity)
      .fieldOr("session_id", lease.session_id.value_or(""), "<absent>")
      .field("error", exc.what())
      .warnThrottle(*clock_, kLogThrottle);
  }
}

}  // namespace livekit_ros2_bridge
