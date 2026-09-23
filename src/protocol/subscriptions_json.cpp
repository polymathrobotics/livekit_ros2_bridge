// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "protocol/subscriptions_json.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol/constants.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
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
    case SubscriptionTargetKind::OtherAudio:
      return "other_audio";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

constexpr char kPayloadField[] = "payload";
constexpr char kSessionIdField[] = "session_id";
constexpr char kSubscriptionsField[] = "subscriptions";
constexpr char kSubscriptionKindField[] = "subscriptions.kind";
constexpr char kSubscriptionNameField[] = "subscriptions.name";
constexpr char kDeliveryPreferencesField[] = "subscriptions.delivery_preferences";
constexpr char kDeliveryPreferencesIntervalMsField[] = "subscriptions.delivery_preferences.interval_ms";
constexpr char kHeartbeatTopicExpansionNodeName[] = "livekit_ros2_bridge";
constexpr char kHeartbeatTopicExpansionNamespace[] = "/";

int parseClampedInt(const nlohmann::json & value, const char * reason)
{
  if (!value.is_number_integer()) {
    throw ValidationError(kDeliveryPreferencesIntervalMsField, reason);
  }

  if (value.is_number_unsigned()) {
    const auto raw = value.get<std::uint64_t>();
    const auto max = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    if (raw > max) {
      return std::numeric_limits<int>::max();
    }
    return static_cast<int>(raw);
  }

  const auto raw = value.get<std::int64_t>();
  const auto min = static_cast<std::int64_t>(std::numeric_limits<int>::min());
  const auto max = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  if (raw < min) {
    return std::numeric_limits<int>::min();
  }
  if (raw > max) {
    return std::numeric_limits<int>::max();
  }

  return static_cast<int>(raw);
}

std::optional<int> parseIntervalMs(const nlohmann::json & entry)
{
  const auto preferences = entry.find("delivery_preferences");
  if (preferences == entry.end()) {
    return std::nullopt;
  }

  if (!preferences->is_object()) {
    throw ValidationError(kDeliveryPreferencesField, "delivery_preferences must be an object");
  }

  const auto interval = preferences->find("interval_ms");
  if (interval == preferences->end()) {
    return std::nullopt;
  }

  const auto ms = parseClampedInt(*interval, "delivery_preferences.interval_ms must be an integer");
  if (ms == 0) {
    return std::nullopt;
  }

  return ms;
}

// Parses one subscription entry into `demand`. Returns false when the entry's
// `kind` is unrecognized; the caller records the entry as unsupported and keeps
// processing the rest of the heartbeat instead of rejecting it. An unrecognized
// entry's remaining fields are echoed but never validated, since the naming rules
// of an unknown kind cannot be assumed. All other validation failures still throw.
bool parseTarget(const nlohmann::json & entry, SubscriptionDemand & demand, UnsupportedSubscription & unsupported)
{
  const auto kind_field = entry.find("kind");
  if (kind_field == entry.end() || !kind_field->is_string()) {
    throw ValidationError(kSubscriptionKindField, "heartbeat subscription 'kind' must be a string");
  }

  const std::string & raw_kind = kind_field->get_ref<const std::string &>();
  const std::string kind = trim(raw_kind);
  if (kind.empty()) {
    throw ValidationError(kSubscriptionKindField, "heartbeat subscription 'kind' must be a non-empty value");
  }
  if (kind == "topic") {
    demand.kind = SubscriptionTargetKind::Topic;
  } else if (kind == "other_video") {
    demand.kind = SubscriptionTargetKind::OtherVideo;
  } else if (kind == "other_audio") {
    demand.kind = SubscriptionTargetKind::OtherAudio;
  } else {
    unsupported.kind = raw_kind;
    if (const auto name_field = entry.find("name"); name_field != entry.end() && name_field->is_string()) {
      unsupported.name = name_field->get_ref<const std::string &>();
    }
    return false;
  }

  const auto name_field = entry.find("name");
  if (name_field == entry.end() || !name_field->is_string()) {
    throw ValidationError(kSubscriptionNameField, "heartbeat subscription 'name' must be a string");
  }

  const auto & raw = name_field->get_ref<const std::string &>();
  if (demand.kind == SubscriptionTargetKind::Topic) {
    const std::string name = trim(raw);
    try {
      demand.name =
        rclcpp::expand_topic_or_service_name(name, kHeartbeatTopicExpansionNodeName, kHeartbeatTopicExpansionNamespace);
    } catch (const std::exception & exc) {
      throw ValidationError(kSubscriptionNameField, exc.what());
    }
    return true;
  }

  demand.name = trim(raw);
  if (demand.name.empty()) {
    throw ValidationError(
      kSubscriptionNameField, "heartbeat subscription other source name must trim to a non-empty name");
  }
  return true;
}

nlohmann::json serialize(const SubscriptionStatus & status)
{
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
  if (status.qos) {
    body["qos"] = {
      {"durability", status.qos->durability},
    };
  }

  const char * delivery_kind = nullptr;
  switch (status.delivery) {
    case SubscriptionDeliveryKind::Data:
      delivery_kind = protocol::kDataDeliveryKind;
      break;
    case SubscriptionDeliveryKind::Video:
      delivery_kind = protocol::kVideoDeliveryKind;
      break;
    case SubscriptionDeliveryKind::Audio:
      delivery_kind = protocol::kAudioDeliveryKind;
      break;
  }
  if (delivery_kind == nullptr) {
    throw std::invalid_argument("subscription delivery kind is invalid");
  }

  nlohmann::json delivery = {
    {"kind", delivery_kind},
    {"track_name", status.track_name},
  };
  if (status.delivery == SubscriptionDeliveryKind::Data) {
    // CDR content type is fixed by the subscription protocol, not caller-selected metadata.
    delivery["content_type"] = protocol::kCdrContentType;
    delivery["interval_ms"] = status.interval_ms;
  }

  body["delivery"] = std::move(delivery);
  return body;
}

nlohmann::json serialize(const SubscriptionErrorStatus & status)
{
  const char * reason = nullptr;
  switch (status.reason) {
    case SubscriptionErrorReason::Forbidden:
      reason = "forbidden";
      break;
    case SubscriptionErrorReason::NotFound:
      reason = "not_found";
      break;
    case SubscriptionErrorReason::UnsupportedKind:
      reason = "unsupported_kind";
      break;
  }
  if (reason == nullptr) {
    throw std::invalid_argument("subscription status error reason is invalid");
  }
  if (status.reason == SubscriptionErrorReason::UnsupportedKind && status.raw_kind.empty()) {
    throw std::invalid_argument("unsupported kind status is missing the raw kind to echo");
  }

  // Unrecognized kinds echo the client's raw kind and name verbatim; recognized kinds
  // use the standard wire mapping.
  nlohmann::json body = {
    {"kind", status.reason == SubscriptionErrorReason::UnsupportedKind ? status.raw_kind : toWire(status.kind)},
    {"status", "error"},
    {"error", {{"reason", reason}, {"message", status.message}}},
  };
  if (status.reason == SubscriptionErrorReason::UnsupportedKind) {
    if (status.raw_name.has_value()) {
      body["name"] = *status.raw_name;
    }
  } else {
    body["name"] = status.name;
  }
  return body;
}

SubscriptionHeartbeat parse(const nlohmann::json & body)
{
  SubscriptionHeartbeat heartbeat;
  std::unordered_map<std::string, std::size_t> index_by_target;
  // Deduplicates unsupported entries by their (raw kind, name) identity, mirroring the
  // first-seen-order `seen`-set idiom used by ros2.interface.show and the config loaders.
  // A pair key keeps absent `name` distinct from an empty-string `name` and is immune to
  // `:` appearing in client-controlled strings.
  std::set<std::pair<std::string, std::optional<std::string>>> seen_unsupported;
  try {
    heartbeat.session_id = protocol::detail::optionalString(
      body, "session_id", "heartbeat session_id must be a string", /*null_is_absent=*/true);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kSessionIdField, exc.what());
  }

  const auto entries = body.find("subscriptions");
  if (entries == body.end()) {
    throw ValidationError(kSubscriptionsField, "heartbeat subscriptions are required");
  }

  if (!entries->is_array()) {
    throw ValidationError(kSubscriptionsField, "heartbeat subscriptions must be an array");
  }

  for (const auto & entry : *entries) {
    if (!entry.is_object()) {
      throw ValidationError(kSubscriptionsField, "heartbeat subscriptions must be objects");
    }

    SubscriptionDemand demand;
    UnsupportedSubscription unsupported;
    if (!parseTarget(entry, demand, unsupported)) {
      // Unrecognized kind: record the entry so the status report can answer it with an
      // `unsupported_kind` error while the bridge keeps honoring the targets it understands.
      if (seen_unsupported.insert({unsupported.kind, unsupported.name}).second) {
        heartbeat.unsupported.push_back(std::move(unsupported));
      }
      continue;
    }
    if (const auto interval = parseIntervalMs(entry)) {
      demand.preferred_interval_ms = *interval;
    }

    const auto [pos, inserted] =
      index_by_target.emplace(std::string(toWire(demand.kind)) + ":" + demand.name, heartbeat.demands.size());
    if (inserted) {
      heartbeat.demands.push_back(std::move(demand));
      continue;
    }

    if (!demand.preferred_interval_ms.has_value()) {
      continue;
    }

    const int requested = *demand.preferred_interval_ms;
    auto & current = heartbeat.demands[pos->second].preferred_interval_ms;
    if (current.has_value() && requested >= *current) {
      continue;
    }

    current = requested;
  }

  return heartbeat;
}

}  // namespace

SubscriptionHeartbeat parse(const std::vector<std::uint8_t> & payload)
{
  try {
    return parse(protocol::detail::parseObject(
      payload, "Invalid JSON in subscription heartbeat", "Subscription heartbeat must be a JSON object"));
  } catch (const ValidationError &) {
    throw;
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayloadField, exc.what());
  }
}

std::string serialize(const SubscriptionStatusReport & report)
{
  nlohmann::json entries = nlohmann::json::array();
  for (const auto & status : report.statuses) {
    entries.push_back(std::visit([](const auto & entry) -> nlohmann::json { return serialize(entry); }, status));
  }

  nlohmann::json body = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    // Protocol compatibility keeps status entries under `subscriptions`.
    {"subscriptions", entries},
  };
  if (report.session_id.has_value()) {
    body["session_id"] = *report.session_id;
  }
  if (report.lease_expiry.has_value()) {
    const auto now = std::chrono::steady_clock::now();
    body["lease_expires_in_ms"] =
      std::chrono::duration_cast<std::chrono::milliseconds>(*report.lease_expiry - now).count();
  }

  return body.dump();
}
}  // namespace livekit_ros2_bridge::protocol::subscriptions
