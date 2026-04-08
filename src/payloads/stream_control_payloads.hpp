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

enum class SubscriptionTargetKind
{
  Topic,
  External,
};

struct SubscriptionTarget
{
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
};

struct SubscriptionRequest
{
  SubscriptionTarget target;
  std::optional<int> preferred_interval_ms;
};

struct SubscriptionHeartbeat
{
  std::optional<std::string> session_id;
  std::vector<SubscriptionRequest> subscriptions;
};

struct StreamStatus
{
  SubscriptionTarget target;
  std::string degraded_reason;
  std::string interface_type;
  int applied_interval_ms = 0;
  std::string delivery_kind;
  std::string publisher_identity;
  std::string track_name;
};

SubscriptionHeartbeat parseSubscriptionHeartbeat(const nlohmann::json & body);

nlohmann::json serializeStreamStatus(const StreamStatus & stream_status);

}  // namespace livekit_ros2_bridge
