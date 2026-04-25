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

enum class SubscriptionTargetKind
{
  Topic,
  OtherVideo,
};

struct SubscriptionDemand
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
  std::optional<int> preferred_interval_ms;
};

struct SubscriptionHeartbeat
{
  // Normalized client-session identifier; absent for missing, null, or blank wire values.
  std::optional<std::string> session_id;
  std::vector<SubscriptionDemand> demands;
};

enum class SubscriptionDeliveryKind
{
  Data,
  Video,
};

struct SubscriptionStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;

  std::string degradation_reason;
  std::string interface_type;

  // Applied data interval; ignored for video delivery.
  int interval_ms = 0;
  SubscriptionDeliveryKind delivery = SubscriptionDeliveryKind::Data;
  std::string track_name;
};

enum class SubscriptionErrorReason
{
  Forbidden,
  NotFound,
};

struct SubscriptionErrorStatus
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
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
