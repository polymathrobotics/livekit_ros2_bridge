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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "utils/json_object_parser.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr int kNoPreferredIntervalOverrideMs = 0;

const char * subscriptionTargetKindString(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::ConfiguredSource:
      return "configured_source";
  }

  throw std::invalid_argument("subscription status target kind is invalid");
}

SubscriptionTargetKind parseSubscriptionTargetKind(std::string_view raw_kind)
{
  const std::string kind = trim(raw_kind);
  if (kind == "topic") {
    return SubscriptionTargetKind::Topic;
  }
  if (kind == "configured_source") {
    return SubscriptionTargetKind::ConfiguredSource;
  }

  throw std::invalid_argument("heartbeat subscription 'kind' must be 'topic' or 'configured_source'");
}

std::string makeSubscriptionTargetKey(const SubscriptionTarget & target)
{
  return std::string(subscriptionTargetKindString(target.kind)) + ":" + target.name;
}

const char * subscriptionDeliveryKindString(SubscriptionDeliveryKind delivery_kind)
{
  switch (delivery_kind) {
    case SubscriptionDeliveryKind::kData:
      return protocol::kDeliveryKindData;
    case SubscriptionDeliveryKind::kVideo:
      return protocol::kDeliveryKindVideo;
  }

  throw std::invalid_argument("subscription status delivery kind is invalid");
}

int parsePreferredIntervalMs(const nlohmann::json & prefs)
{
  const auto interval_it = prefs.find("interval_ms");
  if (interval_it == prefs.end()) {
    return kNoPreferredIntervalOverrideMs;
  }

  if (!interval_it->is_number_integer()) {
    throw std::invalid_argument("delivery_preferences.interval_ms must be an integer");
  }

  if (interval_it->is_number_unsigned()) {
    const auto raw_interval = interval_it->get<std::uint64_t>();
    const auto max_interval = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    return raw_interval > max_interval ? std::numeric_limits<int>::max() : static_cast<int>(raw_interval);
  }

  const auto raw_interval = interval_it->get<std::int64_t>();
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

SubscriptionTarget parseSubscriptionTarget(const nlohmann::json & subscription_json)
{
  const auto kind_it = subscription_json.find("kind");
  if (kind_it == subscription_json.end() || !kind_it->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'kind' must be a string");
  }

  const SubscriptionTargetKind kind = parseSubscriptionTargetKind(kind_it->get_ref<const std::string &>());

  const auto name_it = subscription_json.find("name");
  if (name_it == subscription_json.end() || !name_it->is_string()) {
    throw std::invalid_argument("heartbeat subscription 'name' must be a string");
  }

  std::string name = kind == SubscriptionTargetKind::Topic
                       ? normalizeRosResourceName(name_it->get_ref<const std::string &>())
                       : trimConfiguredSourceName(name_it->get_ref<const std::string &>());
  if (name.empty()) {
    throw std::invalid_argument(
      kind == SubscriptionTargetKind::Topic
        ? "heartbeat subscription topic name must normalize to a non-empty name"
        : "heartbeat subscription configured_source name must trim to a non-empty name");
  }

  return {kind, std::move(name)};
}

SubscriptionDemand parseSubscriptionDemand(const nlohmann::json & subscription_json)
{
  SubscriptionDemand demand;
  demand.target = parseSubscriptionTarget(subscription_json);

  const auto prefs_it = subscription_json.find("delivery_preferences");
  if (prefs_it == subscription_json.end()) {
    return demand;
  }

  if (!prefs_it->is_object()) {
    throw std::invalid_argument("delivery_preferences must be an object");
  }

  const int interval = parsePreferredIntervalMs(*prefs_it);
  if (interval == kNoPreferredIntervalOverrideMs) {
    return demand;
  }

  demand.preferred_interval_ms = interval;
  return demand;
}

}  // namespace

SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body)
{
  SubscriptionHeartbeat heartbeat;
  std::unordered_map<std::string, std::size_t> index_by_key;
  heartbeat.session_id =
    parseOptionalNonEmptyTrimmedStringField(body, "session_id", "heartbeat session_id must be a string", true);
  const auto subs_it = body.find("subscriptions");
  if (subs_it == body.end()) {
    throw std::invalid_argument("heartbeat subscriptions are required");
  }

  if (!subs_it->is_array()) {
    throw std::invalid_argument("heartbeat subscriptions must be an array");
  }

  for (const auto & subscription_json : *subs_it) {
    if (!subscription_json.is_object()) {
      throw std::invalid_argument("heartbeat subscriptions must be objects");
    }

    SubscriptionDemand demand = parseSubscriptionDemand(subscription_json);
    const std::string index_key = makeSubscriptionTargetKey(demand.target);
    const auto [it, inserted] = index_by_key.emplace(index_key, heartbeat.subscriptions.size());
    if (inserted) {
      heartbeat.subscriptions.push_back(std::move(demand));
      continue;
    }

    auto & existing = heartbeat.subscriptions[it->second];
    // Heartbeats carry a demand set keyed by canonical target; repeated demands tighten the
    // interval so the bridge keeps the most demanding subscriber cadence.
    if (demand.preferred_interval_ms.has_value() && existing.preferred_interval_ms.has_value()) {
      existing.preferred_interval_ms = std::min(*existing.preferred_interval_ms, *demand.preferred_interval_ms);
    } else if (demand.preferred_interval_ms.has_value()) {
      existing.preferred_interval_ms = demand.preferred_interval_ms;
    }
  }

  return heartbeat;
}

nlohmann::json serializeSubscriptionStatus(const SubscriptionStatus & subscription_status)
{
  const char * kind_str = subscriptionTargetKindString(subscription_status.target.kind);

  nlohmann::json status_json = {
    {"kind", kind_str},
    {"name", subscription_status.target.name},
    {"status", "active"},
  };

  if (!subscription_status.degraded_reason.empty()) {
    status_json["degraded_reason"] = subscription_status.degraded_reason;
  }
  if (!subscription_status.interface_type.empty()) {
    status_json["interface_type"] = subscription_status.interface_type;
  }

  switch (subscription_status.delivery_kind) {
    case SubscriptionDeliveryKind::kVideo:
      status_json["delivery"] = {
        {"kind", subscriptionDeliveryKindString(subscription_status.delivery_kind)},
        {"track_name", subscription_status.track_name}};
      return status_json;
    case SubscriptionDeliveryKind::kData:
      status_json["delivery"] = {
        {"kind", subscriptionDeliveryKindString(subscription_status.delivery_kind)},
        {"track_name", subscription_status.track_name},
        {"content_type", protocol::kDataContentTypeCdr},
        {"interval_ms", subscription_status.applied_interval_ms}};
      return status_json;
  }

  throw std::invalid_argument("subscription status delivery kind is invalid");
}

}  // namespace livekit_ros2_bridge
