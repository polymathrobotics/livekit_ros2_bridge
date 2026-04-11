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

#include "utils/livekit_access_token.hpp"

#include <openssl/hmac.h>

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "utils/base64.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::optional<std::vector<std::uint8_t>> base64UrlDecode(const std::string & value)
{
  const Base64DecodeResult decoded = livekit_ros2_bridge::base64UrlDecode(value);
  if (!decoded) {
    return std::nullopt;
  }
  return decoded.bytes;
}

std::int64_t toUnixSeconds(std::chrono::system_clock::time_point time_point)
{
  return std::chrono::duration_cast<std::chrono::seconds>(time_point.time_since_epoch()).count();
}

std::optional<nlohmann::json> decodeJwtPayload(const std::string & token)
{
  const auto first_dot = token.find('.');
  if (first_dot == std::string::npos) {
    return std::nullopt;
  }
  const auto second_dot = token.find('.', first_dot + 1U);
  if (second_dot == std::string::npos || second_dot <= first_dot + 1U) {
    return std::nullopt;
  }

  const auto decoded = base64UrlDecode(token.substr(first_dot + 1U, second_dot - first_dot - 1U));
  if (!decoded.has_value()) {
    return std::nullopt;
  }

  try {
    return nlohmann::json::parse(decoded->begin(), decoded->end());
  } catch (const nlohmann::json::exception &) {
    return std::nullopt;
  }
}

}  // namespace

std::string mintLiveKitAccessToken(
  const std::string & api_key,
  const std::string & api_secret,
  const std::string & identity,
  const LiveKitRoomGrant & grant,
  std::chrono::system_clock::time_point issued_at,
  std::chrono::seconds ttl)
{
  if (api_key.empty() || api_secret.empty()) {
    throw std::invalid_argument("LiveKit api_key and api_secret are required.");
  }
  if (identity.empty()) {
    throw std::invalid_argument("LiveKit identity is required.");
  }
  if (grant.room_join && grant.room.empty()) {
    throw std::invalid_argument("LiveKit room is required when room_join is enabled.");
  }

  const auto exp = issued_at + ttl;
  const nlohmann::json header = {
    {"alg", "HS256"},
    {"typ", "JWT"},
  };
  const nlohmann::json video_grant = {
    {"roomJoin", grant.room_join},
    {"room", grant.room},
    {"canPublish", grant.can_publish},
    {"canSubscribe", grant.can_subscribe},
    {"canPublishData", grant.can_publish_data},
  };
  const nlohmann::json payload = {
    {"sub", identity},
    {"iss", api_key},
    {"nbf", toUnixSeconds(issued_at)},
    {"exp", toUnixSeconds(exp)},
    {"video", video_grant},
  };
  const std::string header_json = header.dump();
  const std::string payload_json = payload.dump();

  const std::string encoded_header = livekit_ros2_bridge::base64UrlEncode(
    reinterpret_cast<const std::uint8_t *>(header_json.data()), header_json.size());
  const std::string encoded_payload = livekit_ros2_bridge::base64UrlEncode(
    reinterpret_cast<const std::uint8_t *>(payload_json.data()), payload_json.size());
  const std::string signing_input = encoded_header + "." + encoded_payload;

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (
    HMAC(
      EVP_sha256(),
      api_secret.data(),
      static_cast<int>(api_secret.size()),
      reinterpret_cast<const unsigned char *>(signing_input.data()),
      signing_input.size(),
      digest.data(),
      &digest_size) == nullptr)
  {
    throw std::runtime_error("Failed signing LiveKit access token.");
  }

  const std::string encoded_signature =
    livekit_ros2_bridge::base64UrlEncode(reinterpret_cast<const std::uint8_t *>(digest.data()), digest_size);
  return signing_input + "." + encoded_signature;
}

std::optional<std::chrono::system_clock::time_point> parseJwtExpiresAt(const std::string & token)
{
  const auto payload = decodeJwtPayload(token);
  if (!payload.has_value()) {
    return std::nullopt;
  }

  const auto exp_it = payload->find("exp");
  if (exp_it == payload->end() || !exp_it->is_number_integer()) {
    return std::nullopt;
  }

  const auto exp_seconds = std::chrono::seconds(exp_it->get<std::int64_t>());
  return std::chrono::system_clock::time_point(exp_seconds);
}

}  // namespace livekit_ros2_bridge
