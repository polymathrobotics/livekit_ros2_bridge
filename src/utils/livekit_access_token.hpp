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
  // Room-scoped grant encoded into bridge- or sidecar-minted LiveKit JWTs.
  std::string room;
  bool room_join = true;
  bool can_publish = true;
  bool can_subscribe = true;
  bool can_publish_data = true;
};

// Mints a signed HS256 LiveKit access token for the supplied bridge identity and room grant.
// Callers are responsible for protecting the API secret and for choosing an issued_at/ttl window
// that matches their refresh policy.
std::string mintLiveKitAccessToken(
  const std::string & api_key,
  const std::string & api_secret,
  const std::string & identity,
  const LiveKitRoomGrant & grant,
  std::chrono::system_clock::time_point issued_at,
  std::chrono::seconds ttl);

// Decodes the JWT payload without verifying the signature. Use only for local metadata extraction
// from tokens you already trust, never as an authentication decision.
std::optional<nlohmann::json> decodeJwtPayload(const std::string & token);

// Reads the exp claim via decodeJwtPayload(). Returns nullopt for malformed tokens or missing exp
// and inherits the same "parse only, no signature verification" trust boundary.
std::optional<std::chrono::system_clock::time_point> parseJwtExpiresAt(const std::string & token);

}  // namespace livekit_ros2_bridge
