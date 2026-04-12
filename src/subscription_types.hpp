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
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "protocol.hpp"

namespace livekit_ros2_bridge
{

// Stable subscription target discriminators used in heartbeat and subscription-status payloads.
enum class SubscriptionTargetKind
{
  Topic,
  ConfiguredSource,
};

inline const char * subscriptionTargetKindString(SubscriptionTargetKind kind)
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return "topic";
    case SubscriptionTargetKind::ConfiguredSource:
      return "configured_source";
  }

  throw std::invalid_argument("subscription target kind is invalid");
}

inline std::optional<SubscriptionTargetKind> subscriptionTargetKindFromString(std::string_view kind)
{
  if (kind == "topic") {
    return SubscriptionTargetKind::Topic;
  }
  if (kind == "configured_source") {
    return SubscriptionTargetKind::ConfiguredSource;
  }

  return std::nullopt;
}

// A canonical subscription identifier shared by heartbeat parsing and subscription-status serialization.
struct SubscriptionTarget
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
};

// A single lease-backed subscription demand plus any non-zero delivery preference overrides.
struct SubscriptionDemand
{
  SubscriptionTarget target;
  std::optional<int> preferred_interval_ms;
};

// Parsed form of a subscriptions heartbeat carrying one demand set for a requester.
struct SubscriptionHeartbeat
{
  // Optional trimmed session identifier. Missing, null, or blank values are treated as absent.
  std::optional<std::string> session_id;
  // Wire `subscriptions` array parsed into lease-backed `SubscriptionDemand` objects in
  // first-seen order after coalescing duplicate canonical targets.
  std::vector<SubscriptionDemand> subscriptions;
};

// Delivery mode reported in control-path subscription status. Runtime stream concepts keep
// `stream` naming when they refer to shared runtime resources.
enum class SubscriptionDeliveryKind
{
  kData,
  kVideo,
};

inline const char * subscriptionDeliveryKindString(SubscriptionDeliveryKind delivery_kind)
{
  switch (delivery_kind) {
    case SubscriptionDeliveryKind::kData:
      return protocol::kDeliveryKindData;
    case SubscriptionDeliveryKind::kVideo:
      return protocol::kDeliveryKindVideo;
  }

  throw std::invalid_argument("subscription delivery kind is invalid");
}

// Control-path subscription status object serialized onto `ros.subscriptions.status`.
struct SubscriptionStatus
{
  SubscriptionTarget target;
  // Omitted from the payload when empty.
  std::string degraded_reason;
  // Omitted from the payload when empty.
  std::string interface_type;
  // Serialized only for `data` delivery as `delivery.interval_ms`.
  int applied_interval_ms = 0;
  SubscriptionDeliveryKind delivery_kind = SubscriptionDeliveryKind::kData;
  // Serialized for both delivery modes.
  std::string track_name;
};

}  // namespace livekit_ros2_bridge
