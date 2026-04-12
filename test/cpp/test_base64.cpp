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

void expectCanonicalEncoding(std::initializer_list<std::uint8_t> payload, std::string_view expected_encoded)
{
  const std::vector<std::uint8_t> bytes(payload);
  EXPECT_EQ(base64Encode(bytes.data(), bytes.size()), expected_encoded);

  const Base64DecodeResult decoded = base64Decode(expected_encoded);
  ASSERT_EQ(decoded.status, Base64DecodeStatus::kOk);
  EXPECT_EQ(decoded.bytes, bytes);
}

void expectDecodeRejected(std::string_view encoded, Base64DecodeStatus expected_status)
{
  const Base64DecodeResult decoded = base64Decode(encoded);
  EXPECT_EQ(decoded.status, expected_status);
  EXPECT_TRUE(decoded.bytes.empty());
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
  EXPECT_EQ(base64Encode(nullptr, 0), "");

  const Base64DecodeResult decoded = base64Decode("");
  EXPECT_EQ(decoded.status, Base64DecodeStatus::kOk);
  EXPECT_TRUE(decoded.bytes.empty());
}

TEST(Base64Test, StandardDecodeRejectsMissingPadding)
{
  expectDecodeRejected("AAECAw", Base64DecodeStatus::kMissingPadding);
}

TEST(Base64Test, StandardDecodeRejectsInvalidEncodingSamples)
{
  expectDecodeRejected("AAECAw?=", Base64DecodeStatus::kInvalidEncoding);
  expectDecodeRejected("AAECAw==\n", Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonCanonicalPaddingPlacements)
{
  expectDecodeRejected("A=AA", Base64DecodeStatus::kInvalidEncoding);
  expectDecodeRejected("AA=A", Base64DecodeStatus::kInvalidEncoding);
  expectDecodeRejected("A===", Base64DecodeStatus::kInvalidEncoding);
}

TEST(Base64Test, StandardDecodeRejectsNonZeroTrailingPadBits)
{
  // These decode to the same bytes as AQ== and AQI= unless the decoder validates
  // the unused pad bits in the final quantum.
  expectDecodeRejected("AR==", Base64DecodeStatus::kInvalidEncoding);
  expectDecodeRejected("AQJ=", Base64DecodeStatus::kInvalidEncoding);
}

}  // namespace
}  // namespace livekit_ros2_bridge
