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

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "utils/base64.hpp"

namespace livekit_ros2_bridge
{
namespace
{

TEST(Base64Test, StandardEncodeDecodeRoundTripsAcrossPaddingBoundaries)
{
  const std::vector<std::vector<std::uint8_t>> payloads{
    {},
    {0x01U},
    {0x01U, 0x02U},
    {0x01U, 0x02U, 0x03U},
    {0x01U, 0x02U, 0x03U, 0x04U},
  };

  for (const auto & payload : payloads) {
    const std::string encoded = base64Encode(payload.data(), payload.size());
    const Base64DecodeResult decoded = base64Decode(encoded);
    ASSERT_EQ(decoded.status, Base64DecodeStatus::kOk);
    EXPECT_EQ(decoded.bytes, payload);
  }
}

TEST(Base64Test, StandardDecodeRejectsMissingPadding)
{
  const Base64DecodeResult decoded = base64Decode("AAECAw");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kMissingPadding);
  EXPECT_TRUE(decoded.bytes.empty());
}

TEST(Base64Test, StandardDecodeRejectsInvalidCharacters)
{
  const Base64DecodeResult decoded = base64Decode("AAECAw?=");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsTrailingNewline)
{
  const Base64DecodeResult decoded = base64Decode("AAECAw==\n");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsTrailingCarriageReturnNewline)
{
  const Base64DecodeResult decoded = base64Decode("AAECAw==\r\n");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, UrlEncodeDecodeRoundTripsAcrossPaddingBoundaries)
{
  const std::vector<std::vector<std::uint8_t>> payloads{
    {},
    {0xfbU},
    {0xfbU, 0xefU},
    {0xfbU, 0xefU, 0xffU},
    {0xfbU, 0xefU, 0xffU, 0x01U},
  };

  for (const auto & payload : payloads) {
    const std::string encoded = base64UrlEncode(payload.data(), payload.size());
    const Base64DecodeResult decoded = base64UrlDecode(encoded);
    ASSERT_EQ(decoded.status, Base64DecodeStatus::kOk);
    EXPECT_EQ(decoded.bytes, payload);
  }
}

TEST(Base64Test, UrlDecodeAcceptsUnpaddedInput)
{
  const Base64DecodeResult decoded = base64UrlDecode("AAECAw");
  ASSERT_EQ(decoded.status, Base64DecodeStatus::kOk);
  EXPECT_EQ(decoded.bytes, (std::vector<std::uint8_t>{0x00U, 0x01U, 0x02U, 0x03U}));
}

TEST(Base64Test, UrlDecodeAcceptsPaddedInput)
{
  const Base64DecodeResult decoded = base64UrlDecode("AAECAw==");
  ASSERT_EQ(decoded.status, Base64DecodeStatus::kOk);
  EXPECT_EQ(decoded.bytes, (std::vector<std::uint8_t>{0x00U, 0x01U, 0x02U, 0x03U}));
}

TEST(Base64Test, UrlDecodeRejectsInvalidCharacters)
{
  const Base64DecodeResult decoded = base64UrlDecode("AAECAw?=");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, UrlDecodeRejectsTrailingNewline)
{
  const Base64DecodeResult decoded = base64UrlDecode("AAECAw==\n");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, UrlDecodeRejectsTrailingCarriageReturnNewline)
{
  const Base64DecodeResult decoded = base64UrlDecode("AAECAw==\r\n");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kInvalidEncoding);
}

}  // namespace
}  // namespace livekit_ros2_bridge
