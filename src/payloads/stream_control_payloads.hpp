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

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "nlohmann/json_fwd.hpp"

namespace livekit_ros2_bridge
{

/// Stable subscription target discriminators used in heartbeat and stream-status payloads.
enum class SubscriptionTargetKind
{
  Topic,
  External,
};

/// A normalized subscription identifier. `kind` decides whether the wire field is `topic` or `external`.
struct SubscriptionTarget
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
};

/// A single requested stream plus any non-zero delivery preference overrides.
struct SubscriptionRequest
{
  SubscriptionTarget target;
  std::optional<int> preferred_interval_ms;
};

/// Parsed form of a subscriptions heartbeat.
struct SubscriptionHeartbeat
{
  /// Optional trimmed session identifier. Missing, null, or blank values are treated as absent.
  std::optional<std::string> session_id;
  /// Requested subscriptions in first-seen order after coalescing duplicate normalized targets.
  std::vector<SubscriptionRequest> subscriptions;
};

enum class StreamDeliveryKind
{
  kDataTrack,
  kVideo,
};

/// Stream status entry serialized onto `ros.subscriptions.status`.
struct StreamStatus
{
  SubscriptionTarget target;
  /// Omitted from the payload when empty.
  std::string degraded_reason;
  /// Omitted from the payload when empty.
  std::string interface_type;
  /// Serialized only for `data_track` delivery as `applied_preferences.interval_ms`.
  int applied_interval_ms = 0;
  StreamDeliveryKind delivery_kind = StreamDeliveryKind::kDataTrack;
  /// Serialized for both delivery modes.
  std::string track_name;
};

/// Parse a heartbeat JSON object with optional `session_id` and required `subscriptions`.
/// Each subscription entry must contain exactly one of `topic` or `external`, plus an optional
/// `delivery_preferences.interval_ms` integer. Topic and external names are normalized separately,
/// duplicate normalized targets are coalesced, and duplicate intervals keep the smallest non-zero value.
SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body);

/// Serialize one active stream-status entry. Data-track deliveries include
/// `content_type="application/x-ros-cdr"` and `applied_preferences.interval_ms`.
nlohmann::json serializeStreamStatus(const StreamStatus & stream_status);

}  // namespace livekit_ros2_bridge
