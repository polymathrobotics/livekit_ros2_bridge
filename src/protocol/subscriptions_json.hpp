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

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "protocol/subscriptions.hpp"

namespace livekit_ros2_bridge::protocol::subscriptions
{

/// ROS topic names use rclcpp expansion and validation; `other_video` names are bridge-local.
/// Duplicate canonical targets keep first-seen order and the smallest non-zero interval.
SubscriptionHeartbeat parse(const std::vector<std::uint8_t> & payload);

std::string serialize(const SubscriptionStatusReport & report);

}  // namespace livekit_ros2_bridge::protocol::subscriptions
