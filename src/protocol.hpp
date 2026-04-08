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

inline constexpr char kControlTopicPublish[] = "ros.topics.publish";
inline constexpr char kControlSubscriptionsHeartbeat[] = "ros.subscriptions.heartbeat";
inline constexpr char kControlSubscriptionsStatus[] = "ros.subscriptions.status";
inline constexpr char kRpcServiceCall[] = "ros.services.call";
inline constexpr char kRpcInterfacesGet[] = "ros.interfaces.get";
inline constexpr char kRpcServicesList[] = "ros.services.list";
inline constexpr char kRpcTopicsList[] = "ros.topics.list";
inline constexpr char kDataContentTypeCdr[] = "application/x-ros-cdr";

inline constexpr char kDeliveryKindVideo[] = "video";
inline constexpr char kDeliveryKindDataTrack[] = "data_track";

inline constexpr int kProtocolVersion = 2;

inline constexpr std::uint32_t kRpcErrorInvalidRequest = 2400;
inline constexpr std::uint32_t kRpcErrorUnauthorized = 2401;
inline constexpr std::uint32_t kRpcErrorForbidden = 2403;
inline constexpr std::uint32_t kRpcErrorInternal = 2500;

}  // namespace livekit_ros2_bridge::protocol
