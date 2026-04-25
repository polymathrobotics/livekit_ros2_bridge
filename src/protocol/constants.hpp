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

namespace livekit_ros2_bridge::protocol
{

// These values are wire protocol tokens; change them only with a protocol-version bump.
inline constexpr char kPublishRequestTopic[] = "ros2.topic.pub";
inline constexpr char kHeartbeatTopic[] = "lkros.heartbeat";
inline constexpr char kStatusTopic[] = "lkros.status";
inline constexpr char kCallServiceMethod[] = "ros2.service.call";
inline constexpr char kShowInterfaceMethod[] = "ros2.interface.show";
inline constexpr char kListServicesMethod[] = "ros2.service.list";
inline constexpr char kListTopicsMethod[] = "ros2.topic.list";

inline constexpr char kCdrContentType[] = "application/x-ros-cdr";

inline constexpr char kVideoDeliveryKind[] = "video";
inline constexpr char kDataDeliveryKind[] = "data";

inline constexpr int kProtocolVersion = 2;

inline constexpr std::uint32_t kInvalidRequestRpcCode = 2400;
inline constexpr std::uint32_t kUnauthorizedRpcCode = 2401;
inline constexpr std::uint32_t kForbiddenRpcCode = 2403;
inline constexpr std::uint32_t kInternalRpcCode = 2500;

}  // namespace livekit_ros2_bridge::protocol
