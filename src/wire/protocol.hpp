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

namespace livekit_ros2_bridge::wire::protocol
{

// Stable data-packet topics and RPC names used on the LiveKit data channel. Keep the wire values
// stable even if internal naming changes.
inline constexpr char kRosTopicPublishTopic[] = "ros.topics.publish";
inline constexpr char kSubscriptionsHeartbeatTopic[] = "ros.subscriptions.heartbeat";
inline constexpr char kSubscriptionsStatusTopic[] = "ros.subscriptions.status";
inline constexpr char kRpcServiceCall[] = "ros.services.call";
inline constexpr char kRpcInterfacesGet[] = "ros.interfaces.get";
inline constexpr char kRpcServicesList[] = "ros.services.list";
inline constexpr char kRpcTopicsList[] = "ros.topics.list";

// Stable content-type literal for ROS 2 CDR payload objects.
inline constexpr char kDataContentTypeCdr[] = "application/x-ros-cdr";

// Stable delivery.kind values in subscription status payloads.
inline constexpr char kDeliveryKindVideo[] = "video";
inline constexpr char kDeliveryKindData[] = "data";

// Bump only when the bridge intentionally changes the wire contract.
inline constexpr int kProtocolVersion = 2;

// Stable RPC error codes surfaced to remote callers.
inline constexpr std::uint32_t kRpcErrorInvalidRequest = 2400;
inline constexpr std::uint32_t kRpcErrorUnauthorized = 2401;
inline constexpr std::uint32_t kRpcErrorForbidden = 2403;
inline constexpr std::uint32_t kRpcErrorInternal = 2500;

}  // namespace livekit_ros2_bridge::wire::protocol
