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

#include "protocol/subscriptions_json.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol/constants.hpp"
#include "protocol/detail/json_fields.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::protocol::subscriptions
{

namespace
{

const char * toWire(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::OtherVideo:
      return "other_video";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

const char * toWire(SubscriptionDeliveryKind kind)
{
  switch (kind) {
    case SubscriptionDeliveryKind::Data:
      return protocol::kDataDeliveryKind;
    case SubscriptionDeliveryKind::Video:
      return protocol::kVideoDeliveryKind;
  }

  throw std::invalid_argument("subscription delivery kind is invalid");
}

constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("protocol_subscriptions");

enum class ClampBoundary
{
  None,
  IntMin,
  IntMax,
};

struct ClampedInt
{
  int value;
  ClampBoundary boundary = ClampBoundary::None;
};

rclcpp::Clock & logClock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

const char * boundaryName(ClampBoundary boundary)
{
  switch (boundary) {
    case ClampBoundary::None:
      return "none";
    case ClampBoundary::IntMin:
      return "int_min";
    case ClampBoundary::IntMax:
      return "int_max";
  }

  return "unknown";
}

const char * toWire(SubscriptionStatusErrorReason reason)
{
  switch (reason) {
    case SubscriptionStatusErrorReason::Forbidden:
      return "forbidden";
    case SubscriptionStatusErrorReason::Unavailable:
      return "unavailable";
    case SubscriptionStatusErrorReason::NotFound:
      return "not_found";
  }

  throw std::invalid_argument("subscription status error reason is invalid");
}

ClampedInt parseClampedInt(const nlohmann::json & value, const char * message)
{
  if (!value.is_number_integer()) {
    throw std::invalid_argument(message);
  }

  // Heartbeats can carry JSON integers wider than the local `int` storage used by
  // `SubscriptionDemand`, so clamp before assigning instead of relying on narrowing casts.
  if (value.is_number_unsigned()) {
    const auto raw = value.get<std::uint64_t>();
    const auto max = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    if (raw > max) {
      return {std::numeric_limits<int>::max(), ClampBoundary::IntMax};
    }
    return {static_cast<int>(raw), ClampBoundary::None};
  }

  const auto raw = value.get<std::int64_t>();
  const auto min = static_cast<std::int64_t>(std::numeric_limits<int>::min());
  const auto max = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  if (raw < min) {
    return {std::numeric_limits<int>::min(), ClampBoundary::IntMin};
  }
  if (raw > max) {
    return {std::numeric_limits<int>::max(), ClampBoundary::IntMax};
  }

  return {static_cast<int>(raw), ClampBoundary::None};
}

std::optional<ClampedInt> parseIntervalMs(const nlohmann::json & entry)
{
  const auto preferences = entry.find("delivery_preferences");
  if (preferences == entry.end()) {
    return std::nullopt;
  }

  if (!preferences->is_object()) {
    throw std::invalid_argument("delivery_preferences must be an object");
  }

  const auto interval_it = preferences->find("interval_ms");
  if (interval_it == preferences->end()) {
    return std::nullopt;
  }

  const auto interval = parseClampedInt(*interval_it, "delivery_preferences.interval_ms must be an integer");
  if (interval.value == 0) {
    return std::nullopt;
  }

  return interval;
}

void parseTarget(const nlohmann::json & entry, SubscriptionDemand & demand)
{
  const auto kind_it = entry.find("kind");
  if (kind_it == entry.end() || !kind_it->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be a string");
  }

  const std::string kind = trim(kind_it->get_ref<const std::string &>());
  if (kind == "topic") {
    demand.kind = SubscriptionTargetKind::Topic;
  } else if (kind == "other_video") {
    demand.kind = SubscriptionTargetKind::OtherVideo;
  } else {
    throw std::invalid_argument("heartbeat subscription 'kind' must be 'topic' or 'other_video'");
  }

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

nlohmann::json serialize(const SubscriptionStatus & status)
{
  if (status.delivery != SubscriptionDeliveryKind::Video && status.delivery != SubscriptionDeliveryKind::Data) {
    LogEvent(kLogger, "subscription_status_serialize_failed")
      .field("kind", toWire(status.kind))
      .field("name", status.name)
      .field("delivery_kind", static_cast<int>(status.delivery))
      .error();
    throw std::invalid_argument("subscription status delivery kind is invalid");
  }

  nlohmann::json body = {
    {"kind", toWire(status.kind)},
    {"name", status.name},
    {"status", "active"},
  };

  if (!status.degradation_reason.empty()) {
    body["degraded_reason"] = status.degradation_reason;
  }
  if (!status.interface_type.empty()) {
    body["interface_type"] = status.interface_type;
  }

  nlohmann::json delivery = {
    {"kind", toWire(status.delivery)},
    {"track_name", status.track_name},
  };
  if (status.delivery == SubscriptionDeliveryKind::Data) {
    // Control-path data subscriptions currently transport ROS messages as CDR bytes on a
    // LiveKit data track, so the content type is fixed by protocol rather than caller input.
    delivery["content_type"] = protocol::kCdrContentType;
    delivery["interval_ms"] = status.interval_ms;
  }

  body["delivery"] = std::move(delivery);
  return body;
}

nlohmann::json serialize(const SubscriptionErrorStatus & status)
{
  const char * reason = nullptr;
  try {
    reason = toWire(status.reason);
  } catch (const std::invalid_argument &) {
    LogEvent(kLogger, "subscription_status_serialize_failed")
      .field("kind", toWire(status.kind))
      .field("name", status.name)
      .field("error_reason", static_cast<int>(status.reason))
      .error();
    throw;
  }

  return {
    {"kind", toWire(status.kind)},
    {"name", status.name},
    {"status", "error"},
    {"error", {{"reason", reason}, {"message", status.message}}},
  };
}

SubscriptionHeartbeat parse(const nlohmann::json & body)
{
  SubscriptionHeartbeat heartbeat;
  std::unordered_map<std::string, std::size_t> index_by_key;
  heartbeat.session_id =
    protocol::detail::optionalTrimmedStringField(body, "session_id", "heartbeat session_id must be a string", true);

  const auto entries = body.find("subscriptions");
  if (entries == body.end()) {
    throw std::invalid_argument("heartbeat subscriptions are required");
  }

  if (!entries->is_array()) {
    throw std::invalid_argument("heartbeat subscriptions must be an array");
  }

  for (const auto & entry : *entries) {
    if (!entry.is_object()) {
      throw std::invalid_argument("heartbeat subscriptions must be objects");
    }

    SubscriptionDemand demand;
    parseTarget(entry, demand);
    if (const auto interval = parseIntervalMs(entry)) {
      demand.preferred_interval_ms = interval->value;
      if (interval->boundary != ClampBoundary::None) {
        LogEvent(kLogger, "heartbeat_subscription_interval_clamped")
          .field("kind", toWire(demand.kind))
          .field("name", demand.name)
          .field("boundary", boundaryName(interval->boundary))
          .warnThrottle(logClock(), kLogThrottle);
      }
    }

    // Coalesce within one heartbeat on the canonical `(kind, name)` pair. Topic and
    // other-video identifiers may share the same text, so name alone would alias
    // distinct protocol targets.
    const auto [it, inserted] =
      index_by_key.emplace(std::string(toWire(demand.kind)) + ":" + demand.name, heartbeat.demands.size());
    if (inserted) {
      heartbeat.demands.push_back(std::move(demand));
      continue;
    }

    if (!demand.preferred_interval_ms.has_value()) {
      continue;
    }

    const int requested = *demand.preferred_interval_ms;
    auto & current = heartbeat.demands[it->second].preferred_interval_ms;
    if (current.has_value() && requested >= *current) {
      continue;
    }

    current = requested;
  }

  return heartbeat;
}

}  // namespace

SubscriptionHeartbeat parseHeartbeat(const std::vector<std::uint8_t> & payload)
{
  return parse(
    protocol::detail::parseObject(
      payload, "Invalid JSON in subscription heartbeat", "Subscription heartbeat must be a JSON object"));
}

std::string serializeStatusReport(const SubscriptionStatusReport & report, std::chrono::steady_clock::time_point now)
{
  nlohmann::json entries = nlohmann::json::array();
  for (const auto & status : report.statuses) {
    entries.push_back(std::visit([](const auto & entry) -> nlohmann::json { return serialize(entry); }, status));
  }

  nlohmann::json body = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    // The protocol contract keeps the broad `subscriptions` array name even though each object is one
    // reported subscription-status entry.
    {"subscriptions", entries},
  };
  if (report.session_id.has_value()) {
    body["session_id"] = *report.session_id;
  }
  if (report.lease_expiry.has_value()) {
    body["lease_expires_in_ms"] =
      std::chrono::duration_cast<std::chrono::milliseconds>(*report.lease_expiry - now).count();
  }

  return body.dump();
}

std::string serializeStatusReport(const SubscriptionStatusReport & report)
{
  return serializeStatusReport(report, std::chrono::steady_clock::now());
}
}  // namespace livekit_ros2_bridge::protocol::subscriptions
