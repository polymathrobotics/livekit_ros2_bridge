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
#include "payloads/cdr_base64.hpp"

namespace livekit_ros2_bridge
{
namespace
{

void expectCanonicalEncoding(std::initializer_list<std::uint8_t> payload, std::string_view expected)
{
  const std::vector<std::uint8_t> bytes(payload);
  EXPECT_EQ(wire::cdr::encodeBase64(bytes.data(), bytes.size()), expected);

  const wire::cdr::Base64DecodeResult result = wire::cdr::decodeBase64(expected);
  ASSERT_EQ(result.status, wire::cdr::Base64Status::kOk);
  EXPECT_EQ(result.bytes, bytes);
}

void expectDecodeRejected(std::string_view base64, wire::cdr::Base64Status expected_status)
{
  const wire::cdr::Base64DecodeResult result = wire::cdr::decodeBase64(base64);
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
  EXPECT_EQ(wire::cdr::encodeBase64(nullptr, 0), "");

  const wire::cdr::Base64DecodeResult result = wire::cdr::decodeBase64("");
  EXPECT_EQ(result.status, wire::cdr::Base64Status::kOk);
  EXPECT_TRUE(result.bytes.empty());
}

TEST(Base64Test, StandardDecodeRejectsMissingPadding)
{
  expectDecodeRejected("AAECAw", wire::cdr::Base64Status::kMissingPadding);
}

TEST(Base64Test, StandardDecodeRejectsInvalidEncodingSamples)
{
  expectDecodeRejected("=", wire::cdr::Base64Status::kInvalidEncoding);
  expectDecodeRejected("AAECAw?=", wire::cdr::Base64Status::kInvalidEncoding);
  expectDecodeRejected("AAECAw==\n", wire::cdr::Base64Status::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonCanonicalPaddingPlacements)
{
  expectDecodeRejected("A=AA", wire::cdr::Base64Status::kInvalidEncoding);
  expectDecodeRejected("AA=A", wire::cdr::Base64Status::kInvalidEncoding);
  expectDecodeRejected("A===", wire::cdr::Base64Status::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonZeroTrailingPadBits)
{
  // These decode to the same bytes as AQ== and AQI= unless the decoder validates
  // the unused pad bits in the final quantum.
  expectDecodeRejected("AR==", wire::cdr::Base64Status::kInvalidEncoding);
  expectDecodeRejected("AQJ=", wire::cdr::Base64Status::kInvalidEncoding);
}

}  // namespace
}  // namespace livekit_ros2_bridge
