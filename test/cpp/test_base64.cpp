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
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "protocol/detail/base64.hpp"

namespace livekit_ros2_bridge
{
namespace
{

namespace base64 = protocol::detail::base64;

void expectCanonicalEncoding(std::initializer_list<std::uint8_t> payload, std::string_view expected)
{
  const std::vector<std::uint8_t> bytes(payload);
  EXPECT_EQ(base64::encode(bytes.data(), bytes.size()), expected);

  const base64::Result result = base64::decode(expected);
  ASSERT_EQ(result.status, base64::Status::Ok);
  EXPECT_EQ(result.bytes, bytes);
}

void expectDecodeRejected(std::string_view text, base64::Status expected)
{
  const base64::Result result = base64::decode(text);
  EXPECT_EQ(result.status, expected);
  EXPECT_TRUE(result.bytes.empty());
}

TEST(Base64Test, StandardEncodingMatchesKnownVectorsAcrossPaddingBoundaries)
{
  expectCanonicalEncoding({0x4DU}, "TQ==");
  expectCanonicalEncoding({0x4DU, 0x61U}, "TWE=");
  expectCanonicalEncoding({0x4DU, 0x61U, 0x6EU}, "TWFu");
  expectCanonicalEncoding({0x01U, 0x02U, 0x03U, 0x04U}, "AQIDBA==");
}

TEST(Base64Test, EmptyInputEncodesAndDecodesAsEmpty)
{
  EXPECT_EQ(base64::encode(nullptr, 0), "");

  const base64::Result result = base64::decode("");
  EXPECT_EQ(result.status, base64::Status::Ok);
  EXPECT_TRUE(result.bytes.empty());
}

TEST(Base64Test, StandardDecodeRejectsMissingPadding)
{
  expectDecodeRejected("AAECAw", base64::Status::MissingPadding);
}

TEST(Base64Test, StandardDecodeRejectsInvalidEncodingSamples)
{
  expectDecodeRejected("=", base64::Status::InvalidEncoding);
  expectDecodeRejected("AAECAw?=", base64::Status::InvalidEncoding);
  expectDecodeRejected("AAECAw==\n", base64::Status::InvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonCanonicalPaddingPlacements)
{
  expectDecodeRejected("A=AA", base64::Status::InvalidEncoding);
  expectDecodeRejected("AA=A", base64::Status::InvalidEncoding);
  expectDecodeRejected("A===", base64::Status::InvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonZeroTrailingPadBits)
{
  // Lenient decoders may alias these to AQ== and AQI=.
  expectDecodeRejected("AR==", base64::Status::InvalidEncoding);
  expectDecodeRejected("AQJ=", base64::Status::InvalidEncoding);
}

}  // namespace
}  // namespace livekit_ros2_bridge
