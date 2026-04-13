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

#include "payloads/stream_control_payloads.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "nlohmann/json.hpp"
#include "payloads/json_object_parser.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace stream_control_payloads
{

namespace
{

constexpr auto kNormalizationLogThrottlePeriod = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("stream_control_payloads");

enum class IntegerClampBoundary
{
  kNone,
  kIntMin,
  kIntMax,
};

struct ClampedJsonInteger
{
  int value;
  IntegerClampBoundary boundary = IntegerClampBoundary::kNone;
};

struct PreferredIntervalParseResult
{
  std::optional<int> interval_ms;
  IntegerClampBoundary clamp_boundary = IntegerClampBoundary::kNone;
};

rclcpp::Clock & logClock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

const char * integerClampBoundaryString(IntegerClampBoundary boundary)
{
  switch (boundary) {
    case IntegerClampBoundary::kNone:
      return "none";
    case IntegerClampBoundary::kIntMin:
      return "int_min";
    case IntegerClampBoundary::kIntMax:
      return "int_max";
  }

  return "unknown";
}

ClampedJsonInteger clampJsonIntegerToInt(const nlohmann::json & value, const char * error_message)
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
      return {std::numeric_limits<int>::max(), IntegerClampBoundary::kIntMax};
    }
    return {static_cast<int>(raw_interval), IntegerClampBoundary::kNone};
  }

  const auto raw_interval = value.get<std::int64_t>();
  const auto min_interval = static_cast<std::int64_t>(std::numeric_limits<int>::min());
  const auto max_interval = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  if (raw_interval < min_interval) {
    return {std::numeric_limits<int>::min(), IntegerClampBoundary::kIntMin};
  }
  if (raw_interval > max_interval) {
    return {std::numeric_limits<int>::max(), IntegerClampBoundary::kIntMax};
  }

  return {static_cast<int>(raw_interval), IntegerClampBoundary::kNone};
}

PreferredIntervalParseResult parsePreferredIntervalMs(const nlohmann::json & entry)
{
  const auto delivery_preferences = entry.find("delivery_preferences");
  if (delivery_preferences == entry.end()) {
    return {};
  }

  if (!delivery_preferences->is_object()) {
    throw std::invalid_argument("delivery_preferences must be an object");
  }

  const auto interval_field = delivery_preferences->find("interval_ms");
  if (interval_field == delivery_preferences->end()) {
    return {};
  }

  const auto interval = clampJsonIntegerToInt(*interval_field, "delivery_preferences.interval_ms must be an integer");
  if (interval.value == 0) {
    return {};
  }

  return {interval.value, interval.boundary};
}

SubscriptionTarget parseSubscriptionTarget(const nlohmann::json & entry)
{
  const auto kind_field = entry.find("kind");
  if (kind_field == entry.end() || !kind_field->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be a string");
  }

  const auto parsed_kind = subscriptionTargetKindFromString(trim(kind_field->get_ref<const std::string &>()));
  if (!parsed_kind.has_value()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be 'topic' or 'configured_source'");
  }

  const SubscriptionTargetKind kind = *parsed_kind;
  const auto name_field = entry.find("name");
  if (name_field == entry.end() || !name_field->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'name' must be a string");
  }

  const auto & raw_name = name_field->get_ref<const std::string &>();
  if (kind == SubscriptionTargetKind::Topic) {
    std::string name = normalizeRosResourceName(raw_name);
    if (name.empty()) {
      throw std::invalid_argument("heartbeat subscription topic name must normalize to a non-empty name");
    }

    return {kind, std::move(name)};
  }

  std::string name = trim(raw_name);
  if (name.empty()) {
    throw std::invalid_argument("heartbeat subscription configured_source name must trim to a non-empty name");
  }

  return {kind, std::move(name)};
}

}  // namespace

SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body)
{
  SubscriptionHeartbeat heartbeat;
  std::unordered_map<std::string, std::size_t> demand_index_by_target;
  heartbeat.session_id =
    parseOptionalNonEmptyTrimmedStringField(body, "session_id", "heartbeat session_id must be a string", true);

  const auto subscriptions = body.find("subscriptions");
  if (subscriptions == body.end()) {
    throw std::invalid_argument("heartbeat subscriptions are required");
  }

  if (!subscriptions->is_array()) {
    throw std::invalid_argument("heartbeat subscriptions must be an array");
  }

  for (const auto & entry : *subscriptions) {
    if (!entry.is_object()) {
      throw std::invalid_argument("heartbeat subscriptions must be objects");
    }

    SubscriptionDemand demand;
    demand.target = parseSubscriptionTarget(entry);
    const auto preferred_interval = parsePreferredIntervalMs(entry);
    demand.preferred_interval_ms = preferred_interval.interval_ms;
    if (preferred_interval.clamp_boundary != IntegerClampBoundary::kNone) {
      LogEvent(kLogger, "heartbeat_subscription_interval_clamped")
        .field("kind", subscriptionTargetKindString(demand.target.kind))
        .field("name", demand.target.name)
        .field("boundary", integerClampBoundaryString(preferred_interval.clamp_boundary))
        .warnThrottle(logClock(), kNormalizationLogThrottlePeriod);
    }

    // Coalesce within one heartbeat on the canonical `(kind, name)` pair. Topic and
    // configured_source identifiers may share the same text, so name alone would alias
    // distinct protocol targets.
    const auto [it, inserted] = demand_index_by_target.emplace(
      std::string(subscriptionTargetKindString(demand.target.kind)) + ":" + demand.target.name,
      heartbeat.subscriptions.size());
    if (inserted) {
      heartbeat.subscriptions.push_back(std::move(demand));
      continue;
    }

    if (!demand.preferred_interval_ms.has_value()) {
      continue;
    }

    const int requested_interval = *demand.preferred_interval_ms;
    auto & current_interval = heartbeat.subscriptions[it->second].preferred_interval_ms;
    if (current_interval.has_value() && requested_interval >= *current_interval) {
      continue;
    }

    current_interval = requested_interval;
  }

  return heartbeat;
}

nlohmann::json serializeSubscriptionStatus(const SubscriptionStatus & status)
{
  if (
    status.delivery_kind != SubscriptionDeliveryKind::kVideo && status.delivery_kind != SubscriptionDeliveryKind::kData)
  {
    LogEvent(kLogger, "subscription_status_serialize_failed")
      .field("kind", subscriptionTargetKindString(status.target.kind))
      .field("name", status.target.name)
      .field("delivery_kind", static_cast<int>(status.delivery_kind))
      .error();
    throw std::invalid_argument("subscription status delivery kind is invalid");
  }

  nlohmann::json body = {
    {"kind", subscriptionTargetKindString(status.target.kind)},
    {"name", status.target.name},
    {"status", "active"},
  };

  if (!status.degraded_reason.empty()) {
    body["degraded_reason"] = status.degraded_reason;
  }
  if (!status.interface_type.empty()) {
    body["interface_type"] = status.interface_type;
  }

  nlohmann::json delivery = {
    {"kind", subscriptionDeliveryKindString(status.delivery_kind)},
    {"track_name", status.track_name},
  };
  if (status.delivery_kind == SubscriptionDeliveryKind::kData) {
    // Control-path data subscriptions currently transport ROS messages as CDR bytes on a
    // LiveKit data track, so the content type is fixed by protocol rather than caller input.
    delivery["content_type"] = protocol::kDataContentTypeCdr;
    delivery["interval_ms"] = status.applied_interval_ms;
  }

  body["delivery"] = std::move(delivery);
  return body;
}
}  // namespace stream_control_payloads

}  // namespace livekit_ros2_bridge
