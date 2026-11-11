// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "gtest/gtest.h"
#include "video/h264_access_unit.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint8_t kIdrSlice = 0x65U;
constexpr std::uint8_t kNonIdrSlice = 0x41U;

const Bytes kShortStartCode{0x00U, 0x00U, 0x01U};
const Bytes kLongStartCode{0x00U, 0x00U, 0x00U, 0x01U};

// NAL units (header byte plus a few payload bytes) as an encoder emits them.
const Bytes kAccessUnitDelimiterNal{0x09U, 0x10U};
const Bytes kSequenceParameterSetNal{0x67U, 0x42U, 0xC0U, 0x1EU};
const Bytes kPictureParameterSetNal{0x68U, 0xCEU, 0x3CU, 0x80U};
const Bytes kRecoveryPointSeiNal{0x06U, 0x06U, 0x01U, 0xC4U, 0x80U};
const Bytes kIdrSliceNal{kIdrSlice, 0x88U, 0x84U, 0x00U};
const Bytes kNonIdrSliceNal{kNonIdrSlice, 0x9AU, 0x02U, 0x04U};
const Bytes kNonIdrIntraSliceNal{kNonIdrSlice, 0x88U, 0x84U, 0x00U};

Bytes concat(std::initializer_list<Bytes> parts)
{
  Bytes joined;
  for (const Bytes & part : parts) {
    joined.insert(joined.end(), part.begin(), part.end());
  }
  return joined;
}

bool containsIdr(const Bytes & bytes)
{
  return video::containsIdrSlice(bytes.data(), bytes.size());
}

TEST(H264AccessUnitTest, DetectsIdrAfterParameterSetsWithFourByteStartCodes)
{
  EXPECT_TRUE(containsIdr(concat(
    {kLongStartCode,
     kAccessUnitDelimiterNal,
     kLongStartCode,
     kSequenceParameterSetNal,
     kLongStartCode,
     kPictureParameterSetNal,
     kLongStartCode,
     kIdrSliceNal})));
}

TEST(H264AccessUnitTest, DetectsIdrWithThreeByteStartCodes)
{
  EXPECT_TRUE(containsIdr(concat(
    {kShortStartCode,
     kSequenceParameterSetNal,
     kShortStartCode,
     kPictureParameterSetNal,
     kShortStartCode,
     kIdrSliceNal})));
}

TEST(H264AccessUnitTest, DetectsIdrWhenItIsTheLastByte)
{
  EXPECT_TRUE(containsIdr({0x00U, 0x00U, 0x01U, kIdrSlice}));
}

TEST(H264AccessUnitTest, RejectsNonIdrSlice)
{
  EXPECT_FALSE(containsIdr(concat({kLongStartCode, kNonIdrSliceNal})));
}

TEST(H264AccessUnitTest, RejectsParameterSetsWithoutSlice)
{
  EXPECT_FALSE(
    containsIdr(concat({kLongStartCode, kSequenceParameterSetNal, kLongStartCode, kPictureParameterSetNal})));
}

TEST(H264AccessUnitTest, RejectsRecoveryPointIntraSliceThatIsNotIdr)
{
  // h264parse marks this as a keyframe, but a decoder cannot start from it.
  EXPECT_FALSE(containsIdr(concat(
    {kLongStartCode,
     kSequenceParameterSetNal,
     kLongStartCode,
     kPictureParameterSetNal,
     kLongStartCode,
     kRecoveryPointSeiNal,
     kLongStartCode,
     kNonIdrIntraSliceNal})));
}

TEST(H264AccessUnitTest, RejectsEmptyAndTruncatedInput)
{
  EXPECT_FALSE(video::containsIdrSlice(nullptr, 0U));
  EXPECT_FALSE(containsIdr({}));
  EXPECT_FALSE(containsIdr(kShortStartCode));
  EXPECT_FALSE(containsIdr(kLongStartCode));
  EXPECT_FALSE(containsIdr({0x00U, 0x00U, 0x01U, kNonIdrSlice, 0x9AU, 0x00U, 0x00U}));
  EXPECT_FALSE(containsIdr({0x00U, 0x00U, 0x01U, kNonIdrSlice, 0x9AU, 0x00U}));
}

TEST(H264AccessUnitTest, RejectsIdrHeaderByteWithoutStartCode)
{
  EXPECT_FALSE(containsIdr(kIdrSliceNal));
  EXPECT_FALSE(containsIdr({0x00U, 0x01U, kIdrSlice, 0x88U}));
}

TEST(H264AccessUnitTest, IgnoresIdrHeaderByteBehindEmulationPreventionByte)
{
  // An encoder escapes 00 00 01 inside a payload as 00 00 03 01.
  EXPECT_FALSE(containsIdr({0x00U, 0x00U, 0x01U, kNonIdrSlice, 0x00U, 0x00U, 0x03U, 0x01U, kIdrSlice, 0x00U}));
}

TEST(H264AccessUnitTest, TreatsUnescapedStartCodeInPayloadAsStartCode)
{
  // Annex-B guarantees 00 00 01 never occurs inside a NAL unit, so any
  // occurrence starts a new NAL unit.
  EXPECT_TRUE(containsIdr({0x00U, 0x00U, 0x01U, 0x67U, 0x42U, 0x00U, 0x00U, 0x01U, kIdrSlice, 0x88U}));
}

TEST(H264AccessUnitTest, DecidesAtTheFirstSlice)
{
  // Every slice of one picture has the same NAL type, so the first slice
  // decides. Bytes after a non-IDR slice that look like an IDR NAL unit cannot
  // make the access unit an IDR, and the scan does not read them.
  EXPECT_FALSE(containsIdr(concat({kLongStartCode, kNonIdrSliceNal, kShortStartCode, kIdrSliceNal})));
  EXPECT_TRUE(containsIdr(concat({kLongStartCode, kIdrSliceNal, kShortStartCode, kNonIdrSliceNal})));
}

}  // namespace
}  // namespace livekit_ros2_bridge
