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

#include "nlohmann/json.hpp"

namespace livekit_ros2_bridge
{

struct LiveKitRoomGrant
{
  std::string room;
  bool room_join = true;
  bool can_publish = true;
  bool can_subscribe = true;
  bool can_publish_data = true;
};

std::string mintLiveKitAccessToken(
  const std::string & api_key,
  const std::string & api_secret,
  const std::string & identity,
  const LiveKitRoomGrant & grant,
  std::chrono::system_clock::time_point issued_at,
  std::chrono::seconds ttl);

std::optional<nlohmann::json> decodeJwtPayload(const std::string & token);

std::optional<std::chrono::system_clock::time_point> parseJwtExpiresAt(const std::string & token);

}  // namespace livekit_ros2_bridge
