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

#include "wire/subscriptions.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <variant>

#include "nlohmann/json.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"
#include "wire/detail/json_object_parser.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge::wire::subscriptions
{

const char * targetKindString(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::OtherVideo:
      return "other_video";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

std::optional<SubscriptionTargetKind> targetKindFromString(std::string_view kind)
{
  if (kind == "topic") {
    return SubscriptionTargetKind::Topic;
  }
  if (kind == "other_video") {
    return SubscriptionTargetKind::OtherVideo;
  }

  return std::nullopt;
}

const char * deliveryKindString(SubscriptionDeliveryKind delivery_kind)
{
  switch (delivery_kind) {
    case SubscriptionDeliveryKind::kData:
      return wire::protocol::kDeliveryKindData;
    case SubscriptionDeliveryKind::kVideo:
      return wire::protocol::kDeliveryKindVideo;
  }

  throw std::invalid_argument("subscription delivery kind is invalid");
}

namespace
{

constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("wire_subscriptions");

enum class ClampBoundary
{
  kNone,
  kIntMin,
  kIntMax,
};

struct ClampedInt
{
  int value;
  ClampBoundary boundary = ClampBoundary::kNone;
};

rclcpp::Clock & logClock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

const char * clampBoundaryString(ClampBoundary boundary)
{
  switch (boundary) {
    case ClampBoundary::kNone:
      return "none";
    case ClampBoundary::kIntMin:
      return "int_min";
    case ClampBoundary::kIntMax:
      return "int_max";
  }

  return "unknown";
}

const char * statusErrorReasonString(SubscriptionStatusErrorReason reason)
{
  switch (reason) {
    case SubscriptionStatusErrorReason::kForbidden:
      return "forbidden";
    case SubscriptionStatusErrorReason::kUnavailable:
      return "unavailable";
    case SubscriptionStatusErrorReason::kNotFound:
      return "not_found";
  }

  throw std::invalid_argument("subscription status error reason is invalid");
}

ClampedInt clampJsonInt(const nlohmann::json & value, const char * error_message)
{
  if (!value.is_number_integer()) {
    throw std::invalid_argument(error_message);
  }

  // Heartbeats can carry JSON integers wider than the local `int` storage used by
  // `SubscriptionDemand`, so clamp before assigning instead of relying on narrowing casts.
  if (value.is_number_unsigned()) {
    const auto raw_interval = value.get<std::uint64_t>();
    const auto max_interval = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    if (raw_interval > max_interval) {
      return {std::numeric_limits<int>::max(), ClampBoundary::kIntMax};
    }
    return {static_cast<int>(raw_interval), ClampBoundary::kNone};
  }

  const auto raw_interval = value.get<std::int64_t>();
  const auto min_interval = static_cast<std::int64_t>(std::numeric_limits<int>::min());
  const auto max_interval = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  if (raw_interval < min_interval) {
    return {std::numeric_limits<int>::min(), ClampBoundary::kIntMin};
  }
  if (raw_interval > max_interval) {
    return {std::numeric_limits<int>::max(), ClampBoundary::kIntMax};
  }

  return {static_cast<int>(raw_interval), ClampBoundary::kNone};
}

std::optional<ClampedInt> parseIntervalMs(const nlohmann::json & entry)
{
  const auto delivery_it = entry.find("delivery_preferences");
  if (delivery_it == entry.end()) {
    return std::nullopt;
  }

  if (!delivery_it->is_object()) {
    throw std::invalid_argument("delivery_preferences must be an object");
  }

  const auto interval_it = delivery_it->find("interval_ms");
  if (interval_it == delivery_it->end()) {
    return std::nullopt;
  }

  const auto interval = clampJsonInt(*interval_it, "delivery_preferences.interval_ms must be an integer");
  if (interval.value == 0) {
    return std::nullopt;
  }

  return interval;
}

void parseDemandTarget(const nlohmann::json & entry, SubscriptionDemand & demand)
{
  const auto kind_it = entry.find("kind");
  if (kind_it == entry.end() || !kind_it->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be a string");
  }

  const auto parsed_kind = targetKindFromString(trim(kind_it->get_ref<const std::string &>()));
  if (!parsed_kind.has_value()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be 'topic' or 'other_video'");
  }

  demand.kind = *parsed_kind;
  const auto name_it = entry.find("name");
  if (name_it == entry.end() || !name_it->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'name' must be a string");
  }

  const auto & raw_name = name_it->get_ref<const std::string &>();
  if (demand.kind == SubscriptionTargetKind::Topic) {
    demand.name = normalizeRosResourceName(raw_name);
    if (demand.name.empty()) {
      throw std::invalid_argument("heartbeat subscription topic name must normalize to a non-empty name");
    }
    return;
  }

  demand.name = trim(raw_name);
  if (demand.name.empty()) {
    throw std::invalid_argument("heartbeat subscription other video name must trim to a non-empty name");
  }
}

nlohmann::json serializeSubscriptionStatusEntry(const SubscriptionStatus & status)
{
  if (
    status.delivery_kind != SubscriptionDeliveryKind::kVideo && status.delivery_kind != SubscriptionDeliveryKind::kData)
  {
    LogEvent(kLogger, "subscription_status_serialize_failed")
      .field("kind", targetKindString(status.kind))
      .field("name", status.name)
      .field("delivery_kind", static_cast<int>(status.delivery_kind))
      .error();
    throw std::invalid_argument("subscription status delivery kind is invalid");
  }

  nlohmann::json body = {
    {"kind", targetKindString(status.kind)},
    {"name", status.name},
    {"status", "active"},
  };

  if (!status.degraded_reason.empty()) {
    body["degraded_reason"] = status.degraded_reason;
  }
  if (!status.interface_type.empty()) {
    body["interface_type"] = status.interface_type;
  }

  nlohmann::json delivery = {
    {"kind", deliveryKindString(status.delivery_kind)},
    {"track_name", status.track_name},
  };
  if (status.delivery_kind == SubscriptionDeliveryKind::kData) {
    // Control-path data subscriptions currently transport ROS messages as CDR bytes on a
    // LiveKit data track, so the content type is fixed by protocol rather than caller input.
    delivery["content_type"] = wire::protocol::kDataContentTypeCdr;
    delivery["interval_ms"] = status.applied_interval_ms;
  }

  body["delivery"] = std::move(delivery);
  return body;
}

nlohmann::json serializeSubscriptionStatusEntry(const SubscriptionErrorStatus & status)
{
  const char * reason = nullptr;
  try {
    reason = statusErrorReasonString(status.reason);
  } catch (const std::invalid_argument &) {
    LogEvent(kLogger, "subscription_status_serialize_failed")
      .field("kind", targetKindString(status.kind))
      .field("name", status.name)
      .field("error_reason", static_cast<int>(status.reason))
      .error();
    throw;
  }

  return {
    {"kind", targetKindString(status.kind)},
    {"name", status.name},
    {"status", "error"},
    {"error", {{"reason", reason}, {"message", status.message}}},
  };
}

nlohmann::json serializeSubscriptionStatusEntry(const SubscriptionReportedStatus & status)
{
  return std::visit(
    [](const auto & entry) -> nlohmann::json { return serializeSubscriptionStatusEntry(entry); }, status);
}

}  // namespace

SubscriptionHeartbeat parseHeartbeat(const nlohmann::json & body)
{
  SubscriptionHeartbeat heartbeat;
  std::unordered_map<std::string, std::size_t> index_by_target;
  heartbeat.session_id = wire::detail::parseOptionalNonEmptyTrimmedStringField(
    body, "session_id", "heartbeat session_id must be a string", true);

  const auto subscriptions_it = body.find("subscriptions");
  if (subscriptions_it == body.end()) {
    throw std::invalid_argument("heartbeat subscriptions are required");
  }

  if (!subscriptions_it->is_array()) {
    throw std::invalid_argument("heartbeat subscriptions must be an array");
  }

  for (const auto & entry : *subscriptions_it) {
    if (!entry.is_object()) {
      throw std::invalid_argument("heartbeat subscriptions must be objects");
    }

    SubscriptionDemand demand;
    parseDemandTarget(entry, demand);
    if (const auto interval = parseIntervalMs(entry)) {
      demand.preferred_interval_ms = interval->value;
      if (interval->boundary != ClampBoundary::kNone) {
        LogEvent(kLogger, "heartbeat_subscription_interval_clamped")
          .field("kind", targetKindString(demand.kind))
          .field("name", demand.name)
          .field("boundary", clampBoundaryString(interval->boundary))
          .warnThrottle(logClock(), kLogThrottle);
      }
    }

    // Coalesce within one heartbeat on the canonical `(kind, name)` pair. Topic and
    // other-video identifiers may share the same text, so name alone would alias
    // distinct protocol targets.
    const auto [it, inserted] = index_by_target.emplace(
      std::string(targetKindString(demand.kind)) + ":" + demand.name, heartbeat.subscriptions.size());
    if (inserted) {
      heartbeat.subscriptions.push_back(std::move(demand));
      continue;
    }

    if (!demand.preferred_interval_ms.has_value()) {
      continue;
    }

    const int requested_ms = *demand.preferred_interval_ms;
    auto & current_ms = heartbeat.subscriptions[it->second].preferred_interval_ms;
    if (current_ms.has_value() && requested_ms >= *current_ms) {
      continue;
    }

    current_ms = requested_ms;
  }

  return heartbeat;
}

nlohmann::json serializeStatuses(
  const std::vector<SubscriptionReportedStatus> & statuses,
  const std::optional<std::string> & session_id,
  const std::optional<std::chrono::steady_clock::time_point> & expiry,
  std::chrono::steady_clock::time_point now)
{
  nlohmann::json subscriptions = nlohmann::json::array();
  for (const auto & status : statuses) {
    subscriptions.push_back(serializeSubscriptionStatusEntry(status));
  }

  nlohmann::json body = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kBridgeStatusTopic},
    // The wire contract keeps the broad `subscriptions` array name even though each object is one
    // reported subscription-status entry.
    {"subscriptions", subscriptions},
  };
  if (session_id.has_value()) {
    body["session_id"] = *session_id;
  }
  if (expiry.has_value()) {
    body["lease_expires_in_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(*expiry - now).count();
  }

  return body;
}

nlohmann::json serializeStatuses(
  const std::vector<SubscriptionReportedStatus> & statuses,
  const std::optional<std::string> & session_id,
  const std::optional<std::chrono::steady_clock::time_point> & expiry)
{
  return serializeStatuses(statuses, session_id, expiry, std::chrono::steady_clock::now());
}
}  // namespace livekit_ros2_bridge::wire::subscriptions
