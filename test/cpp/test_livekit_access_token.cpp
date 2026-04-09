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

#include <openssl/evp.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "room_session.hpp"
#include "utils/livekit_access_token.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using Clock = std::chrono::system_clock;

std::optional<std::vector<std::uint8_t>> base64UrlDecode(const std::string & value)
{
  if (value.empty()) {
    return std::vector<std::uint8_t>{};
  }

  std::string padded = value;
  std::replace(padded.begin(), padded.end(), '-', '+');
  std::replace(padded.begin(), padded.end(), '_', '/');
  while ((padded.size() % 4U) != 0U) {
    padded.push_back('=');
  }

  std::vector<std::uint8_t> decoded((padded.size() / 4U) * 3U, 0);
  const int written = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(decoded.data()),
    reinterpret_cast<const unsigned char *>(padded.data()),
    static_cast<int>(padded.size()));
  if (written < 0) {
    return std::nullopt;
  }

  std::size_t actual_size = static_cast<std::size_t>(written);
  if (!padded.empty() && padded.back() == '=') {
    --actual_size;
  }
  if (padded.size() > 1U && padded[padded.size() - 2U] == '=') {
    --actual_size;
  }
  decoded.resize(actual_size);
  return decoded;
}

std::optional<nlohmann::json> decodeJwtPayloadForTest(const std::string & token)
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

TEST(LiveKitAccessTokenTest, MintedTokenIncludesExpectedClaims)
{
  LiveKitRoomGrant grant;
  grant.room = "robot-room";
  const auto issued_at = Clock::time_point(std::chrono::seconds(1700000000));

  const std::string token =
    mintLiveKitAccessToken("api-key", "api-secret", "bridge-id", grant, issued_at, std::chrono::seconds(600));

  const auto payload = decodeJwtPayloadForTest(token);
  ASSERT_TRUE(payload.has_value());
  EXPECT_EQ((*payload)["iss"], "api-key");
  EXPECT_EQ((*payload)["sub"], "bridge-id");
  EXPECT_EQ((*payload)["nbf"], 1700000000);
  EXPECT_EQ((*payload)["exp"], 1700000600);
  EXPECT_EQ((*payload)["video"]["room"], "robot-room");
  EXPECT_TRUE((*payload)["video"]["roomJoin"].get<bool>());
  EXPECT_TRUE((*payload)["video"]["canPublish"].get<bool>());
  EXPECT_TRUE((*payload)["video"]["canSubscribe"].get<bool>());
  EXPECT_TRUE((*payload)["video"]["canPublishData"].get<bool>());
}

TEST(LiveKitAccessTokenTest, ParseJwtExpiresAtReturnsNullForMissingOrMalformedExp)
{
  EXPECT_FALSE(parseJwtExpiresAt("not-a-jwt").has_value());

  const std::string token_without_exp =
    "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
    "eyJpc3MiOiJhcGkta2V5Iiwic3ViIjoiYnJpZGdlLWlkIn0."
    "signature";
  EXPECT_FALSE(parseJwtExpiresAt(token_without_exp).has_value());
}

TEST(LiveKitAccessTokenTest, StaticTokenSourceParsesExpiration)
{
  LiveKitRoomGrant grant;
  grant.room = "robot-room";
  const auto issued_at = Clock::time_point(std::chrono::seconds(1700000000));
  StaticTokenSource source(
    mintLiveKitAccessToken("api-key", "api-secret", "bridge-id", grant, issued_at, std::chrono::seconds(300)));

  RoomConnectionConfig connect_config{"wss://example", "robot-room", "bridge-id"};
  EXPECT_EQ(connect_config.identity, "bridge-id");

  const AccessToken token = source.getToken(connect_config);

  EXPECT_FALSE(token.refreshable);
  EXPECT_FALSE(token.issued_at.has_value());
  ASSERT_TRUE(token.expires_at.has_value());
  EXPECT_EQ(
    std::chrono::duration_cast<std::chrono::seconds>(token.expires_at->time_since_epoch()),
    std::chrono::seconds(1700000300));
}

TEST(LiveKitAccessTokenTest, ApiKeyAccessTokenSourceMarksTokensRefreshable)
{
  ApiKeyAccessTokenSource source("api-key", "api-secret", std::chrono::seconds(900));
  RoomConnectionConfig connect_config{"wss://example", "robot-room", "bridge-id"};

  EXPECT_EQ(connect_config.identity, "bridge-id");

  const AccessToken token = source.getToken(connect_config);

  EXPECT_TRUE(token.refreshable);
  ASSERT_TRUE(token.issued_at.has_value());
  ASSERT_TRUE(token.expires_at.has_value());
  EXPECT_EQ(
    std::chrono::duration_cast<std::chrono::seconds>(*token.expires_at - *token.issued_at), std::chrono::seconds(900));

  const auto payload = decodeJwtPayloadForTest(token.value);
  ASSERT_TRUE(payload.has_value());
  EXPECT_EQ((*payload)["iss"], "api-key");
  EXPECT_EQ((*payload)["sub"], "bridge-id");
  EXPECT_EQ((*payload)["video"]["room"], "robot-room");
}

TEST(LiveKitAccessTokenTest, MintRejectsEmptyApiKey)
{
  LiveKitRoomGrant grant;
  grant.room = "robot-room";
  EXPECT_THROW(
    mintLiveKitAccessToken("", "api-secret", "bridge-id", grant, Clock::now(), std::chrono::seconds(600)),
    std::invalid_argument);
}

TEST(LiveKitAccessTokenTest, MintRejectsEmptyApiSecret)
{
  LiveKitRoomGrant grant;
  grant.room = "robot-room";
  EXPECT_THROW(
    mintLiveKitAccessToken("api-key", "", "bridge-id", grant, Clock::now(), std::chrono::seconds(600)),
    std::invalid_argument);
}

TEST(LiveKitAccessTokenTest, MintRejectsEmptyIdentity)
{
  LiveKitRoomGrant grant;
  grant.room = "robot-room";
  EXPECT_THROW(
    mintLiveKitAccessToken("api-key", "api-secret", "", grant, Clock::now(), std::chrono::seconds(600)),
    std::invalid_argument);
}

TEST(LiveKitAccessTokenTest, MintRejectsEmptyRoomWhenRoomJoinEnabled)
{
  LiveKitRoomGrant grant;
  grant.room_join = true;
  grant.room = "";
  EXPECT_THROW(
    mintLiveKitAccessToken("api-key", "api-secret", "bridge-id", grant, Clock::now(), std::chrono::seconds(600)),
    std::invalid_argument);
}

TEST(LiveKitAccessTokenTest, MintAllowsEmptyRoomWhenRoomJoinDisabled)
{
  LiveKitRoomGrant grant;
  grant.room_join = false;
  grant.room = "";
  EXPECT_NO_THROW(
    mintLiveKitAccessToken("api-key", "api-secret", "bridge-id", grant, Clock::now(), std::chrono::seconds(600)));
}

}  // namespace
}  // namespace livekit_ros2_bridge
