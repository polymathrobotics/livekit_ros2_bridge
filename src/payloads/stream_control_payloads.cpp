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

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "nlohmann/json.hpp"
#include "payloads/json_object_parser.hpp"
#include "protocol.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace stream_control_payloads
{

namespace
{

int clampJsonIntegerToInt(const nlohmann::json & value, const char * error_message)
{
  if (!value.is_number_integer()) {
    throw std::invalid_argument(error_message);
  }

  // Heartbeats can carry JSON integers wider than the local `int` storage used by
  // `SubscriptionDemand`, so clamp before assigning instead of relying on narrowing casts.
  if (value.is_number_unsigned()) {
    const auto raw_interval = value.get<std::uint64_t>();
    const auto max_interval = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    return raw_interval > max_interval ? std::numeric_limits<int>::max() : static_cast<int>(raw_interval);
  }

  const auto raw_interval = value.get<std::int64_t>();
  const auto min_interval = static_cast<std::int64_t>(std::numeric_limits<int>::min());
  const auto max_interval = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  if (raw_interval < min_interval) {
    return std::numeric_limits<int>::min();
  }
  if (raw_interval > max_interval) {
    return std::numeric_limits<int>::max();
  }

  return static_cast<int>(raw_interval);
}

std::optional<int> parsePreferredIntervalMs(const nlohmann::json & entry)
{
  const auto delivery_preferences = entry.find("delivery_preferences");
  if (delivery_preferences == entry.end()) {
    return std::nullopt;
  }

  if (!delivery_preferences->is_object()) {
    throw std::invalid_argument("delivery_preferences must be an object");
  }

  const auto interval_field = delivery_preferences->find("interval_ms");
  if (interval_field == delivery_preferences->end()) {
    return std::nullopt;
  }

  const int interval_ms = clampJsonIntegerToInt(*interval_field, "delivery_preferences.interval_ms must be an integer");
  if (interval_ms == 0) {
    return std::nullopt;
  }

  return interval_ms;
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

  std::string name = kind == SubscriptionTargetKind::Topic
                       ? normalizeRosResourceName(name_field->get_ref<const std::string &>())
                       : trim(name_field->get_ref<const std::string &>());
  if (name.empty()) {
    throw std::invalid_argument(
      kind == SubscriptionTargetKind::Topic
        ? "heartbeat subscription topic name must normalize to a non-empty name"
        : "heartbeat subscription configured_source name must trim to a non-empty name");
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
    demand.preferred_interval_ms = parsePreferredIntervalMs(entry);

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

    auto & current = heartbeat.subscriptions[it->second];
    if (!current.preferred_interval_ms.has_value() || *demand.preferred_interval_ms < *current.preferred_interval_ms) {
      current.preferred_interval_ms = *demand.preferred_interval_ms;
    }
  }

  return heartbeat;
}

nlohmann::json serializeSubscriptionStatus(const SubscriptionStatus & status)
{
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

  switch (status.delivery_kind) {
    case SubscriptionDeliveryKind::kVideo:
      body["delivery"] = {
        {"kind", subscriptionDeliveryKindString(status.delivery_kind)}, {"track_name", status.track_name}};
      return body;
    case SubscriptionDeliveryKind::kData:
      // Control-path data subscriptions currently transport ROS messages as CDR bytes on a
      // LiveKit data track, so the content type is fixed by protocol rather than caller input.
      body["delivery"] = {
        {"kind", subscriptionDeliveryKindString(status.delivery_kind)},
        {"track_name", status.track_name},
        {"content_type", protocol::kDataContentTypeCdr},
        {"interval_ms", status.applied_interval_ms}};
      return body;
  }

  throw std::invalid_argument("subscription status delivery kind is invalid");
}
}  // namespace stream_control_payloads

}  // namespace livekit_ros2_bridge
