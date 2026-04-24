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

#include <chrono>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace livekit_ros2_bridge
{

// Stable subscription target categories shared by lease management and subscription protocol codecs.
enum class SubscriptionTargetKind
{
  Topic,
  OtherVideo,
};

// A single lease-backed subscription demand plus any non-zero delivery preference overrides.
struct SubscriptionDemand
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
  std::optional<int> preferred_interval_ms;
};

// Lease-backed subscription demand set for a requester.
struct SubscriptionHeartbeat
{
  // Optional normalized client-session identifier.
  std::optional<std::string> session_id;
  // First-seen lease-backed demands after coalescing duplicate canonical targets.
  std::vector<SubscriptionDemand> demands;
};

// Delivery mode reported for an active subscription. Runtime stream concepts keep `stream` naming
// when they refer to shared runtime resources.
enum class SubscriptionDeliveryKind
{
  Data,
  Video,
};

// Active subscription status assembled by runtime code before protocol serialization.
struct SubscriptionStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;

  // Empty when no degradation is being reported.
  std::string degradation_reason;
  // Empty when no ROS interface type applies or is available.
  std::string interface_type;

  // Applied data delivery interval. Ignored for video delivery.
  int interval_ms = 0;
  SubscriptionDeliveryKind delivery = SubscriptionDeliveryKind::Data;
  // Data or media track carrying this subscription.
  std::string track_name;
};

enum class SubscriptionStatusErrorReason
{
  Forbidden,
  Unavailable,
  NotFound,
};

// Error subscription status assembled by runtime code before protocol serialization.
struct SubscriptionErrorStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
  SubscriptionStatusErrorReason reason = SubscriptionStatusErrorReason::NotFound;
  std::string message;
};

using SubscriptionReportedStatus = std::variant<SubscriptionStatus, SubscriptionErrorStatus>;

// Complete typed status report before it crosses the subscription-status protocol boundary.
struct SubscriptionStatusReport
{
  std::vector<SubscriptionReportedStatus> statuses;
  std::optional<std::string> session_id;
  std::optional<std::chrono::steady_clock::time_point> lease_expiry;
};

}  // namespace livekit_ros2_bridge
