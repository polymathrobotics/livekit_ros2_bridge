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

#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace livekit_ros2_bridge
{

enum class SubscriptionTargetKind
{
  Topic,
  OtherVideo,
  OtherAudio,
};

struct SubscriptionDemand
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
  std::optional<int> preferred_interval_ms;
};

// A heartbeat entry whose `kind` this bridge does not recognize. The raw kind is
// echoed verbatim so the client can correlate the entry with its own request; the
// name is echoed only when the client sent a string, since an unknown kind's name
// semantics cannot be assumed.
struct UnsupportedSubscription
{
  std::string kind;
  std::optional<std::string> name;
};

struct SubscriptionHeartbeat
{
  // Normalized client-session identifier; absent for missing, null, or blank wire values.
  std::optional<std::string> session_id;
  std::vector<SubscriptionDemand> demands;
  // Unrecognized-kind entries, deduplicated by (raw kind, name) in first-seen order.
  std::vector<UnsupportedSubscription> unsupported;
};

enum class SubscriptionDeliveryKind
{
  Data,
  Video,
  Audio,
};

struct SubscriptionQos
{
  std::string durability;  // "volatile" | "transient_local"
};

struct SubscriptionStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;

  std::string degradation_reason;
  std::string interface_type;

  std::optional<SubscriptionQos> qos;

  // Applied data interval; ignored for video delivery.
  int interval_ms = 0;
  SubscriptionDeliveryKind delivery = SubscriptionDeliveryKind::Data;
  std::string track_name;
};

enum class SubscriptionErrorReason
{
  Forbidden,
  NotFound,
  UnsupportedKind,
};

// Error statuses echo the request's `kind` and `name` back to the client. Recognized
// kinds carry their enum and resolved name, serialized through the standard wire
// mapping; `UnsupportedKind` carries the client's raw kind string verbatim and echoes
// the name only when the client sent one.
struct SubscriptionErrorStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
  // Raw wire `kind` for `UnsupportedKind`; empty for recognized kinds.
  std::string raw_kind;
  // Raw wire `name` echoed verbatim for `UnsupportedKind`; nullopt when the client sent
  // no string name (or the entry has a recognized kind).
  std::optional<std::string> raw_name;
  SubscriptionErrorReason reason = SubscriptionErrorReason::NotFound;
  std::string message;
};

using SubscriptionStatusEntry = std::variant<SubscriptionStatus, SubscriptionErrorStatus>;

struct SubscriptionStatusReport
{
  std::vector<SubscriptionStatusEntry> statuses;
  std::optional<std::string> session_id;
  // Steady-clock expiry converted to relative `lease_expires_in_ms` during serialization.
  std::optional<std::chrono::steady_clock::time_point> lease_expiry;
};

}  // namespace livekit_ros2_bridge
