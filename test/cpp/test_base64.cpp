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
#include "wire/detail/base64.hpp"

namespace livekit_ros2_bridge
{
namespace
{

void expectCanonicalEncoding(std::initializer_list<std::uint8_t> payload, std::string_view expected)
{
  const std::vector<std::uint8_t> bytes(payload);
  EXPECT_EQ(wire::detail::encodeBase64(bytes.data(), bytes.size()), expected);

  const wire::detail::Base64DecodeResult result = wire::detail::decodeBase64(expected);
  ASSERT_EQ(result.status, wire::detail::Base64Status::Ok);
  EXPECT_EQ(result.bytes, bytes);
}

void expectDecodeRejected(std::string_view base64, wire::detail::Base64Status expected_status)
{
  const wire::detail::Base64DecodeResult result = wire::detail::decodeBase64(base64);
  EXPECT_EQ(result.status, expected_status);
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
  EXPECT_EQ(wire::detail::encodeBase64(nullptr, 0), "");

  const wire::detail::Base64DecodeResult result = wire::detail::decodeBase64("");
  EXPECT_EQ(result.status, wire::detail::Base64Status::Ok);
  EXPECT_TRUE(result.bytes.empty());
}

TEST(Base64Test, StandardDecodeRejectsMissingPadding)
{
  expectDecodeRejected("AAECAw", wire::detail::Base64Status::MissingPadding);
}

TEST(Base64Test, StandardDecodeRejectsInvalidEncodingSamples)
{
  expectDecodeRejected("=", wire::detail::Base64Status::InvalidEncoding);
  expectDecodeRejected("AAECAw?=", wire::detail::Base64Status::InvalidEncoding);
  expectDecodeRejected("AAECAw==\n", wire::detail::Base64Status::InvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonCanonicalPaddingPlacements)
{
  expectDecodeRejected("A=AA", wire::detail::Base64Status::InvalidEncoding);
  expectDecodeRejected("AA=A", wire::detail::Base64Status::InvalidEncoding);
  expectDecodeRejected("A===", wire::detail::Base64Status::InvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonZeroTrailingPadBits)
{
  // These decode to the same bytes as AQ== and AQI= unless the decoder validates
  // the unused pad bits in the final quantum.
  expectDecodeRejected("AR==", wire::detail::Base64Status::InvalidEncoding);
  expectDecodeRejected("AQJ=", wire::detail::Base64Status::InvalidEncoding);
}

}  // namespace
}  // namespace livekit_ros2_bridge
