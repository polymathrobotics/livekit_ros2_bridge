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
#include "video/access_unit.hpp"

namespace livekit_ros2_bridge::video
{
namespace
{

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint8_t kIdrSlice = 0x65U;
constexpr std::uint8_t kNonIdrSlice = 0x41U;

const Bytes kShortStartCode{0x00U, 0x00U, 0x01U};
const Bytes kLongStartCode{0x00U, 0x00U, 0x00U, 0x01U};

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

AccessUnitSummary summarize(const Bytes & bytes)
{
  return summarizeH264AccessUnit(bytes.data(), bytes.size());
}

bool containsIdr(const Bytes & bytes)
{
  return summarize(bytes).is_key_frame;
}

TEST(H264AccessUnitTest, DetectsIdrWhenItIsTheLastByte)
{
  EXPECT_TRUE(containsIdr({0x00U, 0x00U, 0x01U, kIdrSlice}));
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
  EXPECT_FALSE(summarizeH264AccessUnit(nullptr, 0U).is_key_frame);
  EXPECT_FALSE(containsIdr(kShortStartCode));
  EXPECT_FALSE(containsIdr(kLongStartCode));
  // Ends at a start code; the IDR header byte just past `size` must not be read.
  const Bytes start_code_at_end = concat({kLongStartCode, kSequenceParameterSetNal, kShortStartCode, {kIdrSlice}});
  EXPECT_FALSE(summarizeH264AccessUnit(start_code_at_end.data(), start_code_at_end.size() - 1U).is_key_frame);
}

TEST(H264AccessUnitTest, IgnoresIdrHeaderByteNotAfterStartCode)
{
  EXPECT_FALSE(containsIdr(kIdrSliceNal));
  EXPECT_FALSE(containsIdr({0x00U, 0x01U, kIdrSlice, 0x88U}));
  // 00 00 03 01 is an escaped 00 00 01 inside a payload, not a NAL unit start;
  // a parameter set, unlike a slice, lets the scan reach it.
  EXPECT_FALSE(containsIdr({0x00U, 0x00U, 0x01U, 0x67U, 0x42U, 0x00U, 0x00U, 0x03U, 0x01U, kIdrSlice, 0x00U}));
}

TEST(H264AccessUnitTest, DecidesAtTheFirstSlice)
{
  // All slices of a picture share a NAL type, so the scan must not read past the first slice.
  EXPECT_FALSE(containsIdr(concat({kLongStartCode, kNonIdrSliceNal, kShortStartCode, kIdrSliceNal})));
  EXPECT_TRUE(containsIdr(concat({kLongStartCode, kIdrSliceNal, kShortStartCode, kNonIdrSliceNal})));
}

TEST(H264AccessUnitTest, ReportsParameterSetsOnlyWhenSpsAndPpsBothPrecedeTheIdrSlice)
{
  const AccessUnitSummary with_both = summarize(concat(
    {kLongStartCode,
     kAccessUnitDelimiterNal,
     kLongStartCode,
     kSequenceParameterSetNal,
     kShortStartCode,
     kPictureParameterSetNal,
     kShortStartCode,
     kIdrSliceNal}));
  EXPECT_TRUE(with_both.is_key_frame);
  EXPECT_TRUE(with_both.has_parameter_sets);

  const AccessUnitSummary without_either =
    summarize(concat({kLongStartCode, kAccessUnitDelimiterNal, kLongStartCode, kIdrSliceNal}));
  EXPECT_TRUE(without_either.is_key_frame);
  EXPECT_FALSE(without_either.has_parameter_sets);

  const AccessUnitSummary sps_only =
    summarize(concat({kLongStartCode, kSequenceParameterSetNal, kLongStartCode, kIdrSliceNal}));
  EXPECT_TRUE(sps_only.is_key_frame);
  EXPECT_FALSE(sps_only.has_parameter_sets);

  const AccessUnitSummary pps_only =
    summarize(concat({kLongStartCode, kPictureParameterSetNal, kLongStartCode, kIdrSliceNal}));
  EXPECT_TRUE(pps_only.is_key_frame);
  EXPECT_FALSE(pps_only.has_parameter_sets);

  const AccessUnitSummary after_slice = summarize(concat(
    {kLongStartCode,
     kIdrSliceNal,
     kShortStartCode,
     kSequenceParameterSetNal,
     kShortStartCode,
     kPictureParameterSetNal}));
  EXPECT_TRUE(after_slice.is_key_frame);
  EXPECT_FALSE(after_slice.has_parameter_sets);
}

// H.265 NAL header: two bytes, (type << 1) then 0x01.
const Bytes kH265AccessUnitDelimiterNal{0x46U, 0x01U, 0x50U};
const Bytes kH265VideoParameterSetNal{0x40U, 0x01U, 0x0CU, 0x01U};
const Bytes kH265SequenceParameterSetNal{0x42U, 0x01U, 0x01U, 0x01U};
const Bytes kH265PictureParameterSetNal{0x44U, 0x01U, 0xC1U, 0x72U};
const Bytes kH265IdrWithLeadingPicturesNal{0x26U, 0x01U, 0xAFU, 0x08U};
const Bytes kH265IdrWithoutLeadingPicturesNal{0x28U, 0x01U, 0xAFU, 0x08U};
const Bytes kH265CleanRandomAccessNal{0x2AU, 0x01U, 0xAFU, 0x08U};
const Bytes kH265TrailingPictureNal{0x02U, 0x01U, 0xD0U, 0x08U};

AccessUnitSummary summarizeH265(const Bytes & bytes)
{
  return summarizeH265AccessUnit(bytes.data(), bytes.size());
}

Bytes h265AccessUnitWithParameterSets(const Bytes & slice)
{
  return concat(
    {kLongStartCode,
     kH265AccessUnitDelimiterNal,
     kLongStartCode,
     kH265VideoParameterSetNal,
     kShortStartCode,
     kH265SequenceParameterSetNal,
     kShortStartCode,
     kH265PictureParameterSetNal,
     kShortStartCode,
     slice});
}

TEST(H265AccessUnitTest, DetectsBothIdrTypesWithTheirParameterSets)
{
  for (const Bytes & idr_slice : {kH265IdrWithLeadingPicturesNal, kH265IdrWithoutLeadingPicturesNal}) {
    SCOPED_TRACE(static_cast<int>(idr_slice.front()));
    const AccessUnitSummary summary = summarizeH265(h265AccessUnitWithParameterSets(idr_slice));
    EXPECT_TRUE(summary.is_key_frame);
    EXPECT_TRUE(summary.has_parameter_sets);
  }
}

TEST(H265AccessUnitTest, ReportsParameterSetsOnlyWhenVpsSpsAndPpsAllPrecedeTheIdrSlice)
{
  const AccessUnitSummary without_vps = summarizeH265(concat(
    {kLongStartCode,
     kH265SequenceParameterSetNal,
     kLongStartCode,
     kH265PictureParameterSetNal,
     kLongStartCode,
     kH265IdrWithLeadingPicturesNal}));
  EXPECT_TRUE(without_vps.is_key_frame);
  EXPECT_FALSE(without_vps.has_parameter_sets);

  const AccessUnitSummary without_sps = summarizeH265(concat(
    {kLongStartCode,
     kH265VideoParameterSetNal,
     kLongStartCode,
     kH265PictureParameterSetNal,
     kLongStartCode,
     kH265IdrWithLeadingPicturesNal}));
  EXPECT_TRUE(without_sps.is_key_frame);
  EXPECT_FALSE(without_sps.has_parameter_sets);

  const AccessUnitSummary without_pps = summarizeH265(concat(
    {kLongStartCode,
     kH265VideoParameterSetNal,
     kLongStartCode,
     kH265SequenceParameterSetNal,
     kLongStartCode,
     kH265IdrWithLeadingPicturesNal}));
  EXPECT_TRUE(without_pps.is_key_frame);
  EXPECT_FALSE(without_pps.has_parameter_sets);

  const AccessUnitSummary after_slice = summarizeH265(concat(
    {kLongStartCode,
     kH265IdrWithLeadingPicturesNal,
     kLongStartCode,
     kH265VideoParameterSetNal,
     kLongStartCode,
     kH265SequenceParameterSetNal,
     kLongStartCode,
     kH265PictureParameterSetNal}));
  EXPECT_TRUE(after_slice.is_key_frame);
  EXPECT_FALSE(after_slice.has_parameter_sets);
}

TEST(H265AccessUnitTest, RejectsCleanRandomAccessAndTrailingPictures)
{
  // A CRA's leading pictures can't be decoded from a fresh start, so it isn't a keyframe.
  EXPECT_FALSE(summarizeH265(h265AccessUnitWithParameterSets(kH265CleanRandomAccessNal)).is_key_frame);
  EXPECT_FALSE(summarizeH265(h265AccessUnitWithParameterSets(kH265TrailingPictureNal)).is_key_frame);
  EXPECT_FALSE(
    summarizeH265(concat({kLongStartCode, kH265TrailingPictureNal, kShortStartCode, kH265IdrWithLeadingPicturesNal}))
      .is_key_frame);
}

TEST(H265AccessUnitTest, ReadsTheTypeFromItsOwnHeaderLayout)
{
  const Bytes idr_header_at_end{0x00U, 0x00U, 0x01U, 0x26U};
  EXPECT_TRUE(summarizeH265(idr_header_at_end).is_key_frame);
  // H.264 reads 0x26 as an SEI, and H.265 reads H.264's IDR header byte as type 50.
  EXPECT_FALSE(containsIdr(idr_header_at_end));
  EXPECT_FALSE(summarizeH265({0x00U, 0x00U, 0x01U, kIdrSlice}).is_key_frame);
  EXPECT_FALSE(summarizeH265AccessUnit(nullptr, 0U).is_key_frame);
}

// OBU header: (type << 3) | 0x02 when it has a size field, then a leb128 size.
constexpr std::uint8_t kSequenceHeaderObu = 0x0AU;
constexpr std::uint8_t kFrameObu = 0x32U;
constexpr std::uint8_t kHasExtensionBit = 0x04U;
const Bytes kTemporalDelimiterObu{0x12U, 0x00U};
const Bytes kSequenceHeaderObuWithPayload{kSequenceHeaderObu, 0x03U, 0x00U, 0x00U, 0x00U};
const Bytes kFrameObuWithPayload{kFrameObu, 0x02U, 0x10U, 0x00U};

bool hasSequenceHeader(const Bytes & bytes)
{
  return hasAv1SequenceHeader(bytes.data(), bytes.size());
}

TEST(Av1TemporalUnitTest, FindsTheSequenceHeaderAfterTheTemporalDelimiter)
{
  EXPECT_TRUE(hasSequenceHeader(concat({kTemporalDelimiterObu, kSequenceHeaderObuWithPayload, kFrameObuWithPayload})));
  EXPECT_FALSE(hasSequenceHeader(concat({kTemporalDelimiterObu, kFrameObuWithPayload})));
}

TEST(Av1TemporalUnitTest, SkipsExtensionBytesAndMultiByteSizes)
{
  // A 128-byte frame OBU with an extension byte; its size is the two-byte leb128 80 01.
  Bytes frame_with_extension{kFrameObu | kHasExtensionBit, 0x08U, 0x80U, 0x01U};
  frame_with_extension.resize(frame_with_extension.size() + 128U, 0x00U);

  EXPECT_TRUE(hasSequenceHeader(concat({frame_with_extension, kSequenceHeaderObuWithPayload})));
}

TEST(Av1TemporalUnitTest, StopsSafelyAtTruncatedOrMalformedObus)
{
  EXPECT_FALSE(hasAv1SequenceHeader(nullptr, 0U));
  // The sequence header claims more bytes than the buffer holds.
  EXPECT_FALSE(hasSequenceHeader({kSequenceHeaderObu, 0x05U, 0x00U}));
  // The frame's size runs past the sequence header that follows it.
  EXPECT_FALSE(hasSequenceHeader(concat({{kFrameObu, 0x09U, 0x00U}, kSequenceHeaderObuWithPayload})));
  // A size that never ends, an extension byte missing, and a size field missing at the end.
  EXPECT_FALSE(hasSequenceHeader({kFrameObu, 0x80U, 0x80U}));
  EXPECT_FALSE(hasSequenceHeader({kFrameObu | kHasExtensionBit}));
  EXPECT_FALSE(hasSequenceHeader({kFrameObu}));
  // With no size field the OBU runs to the end, so the sequence header bytes are its payload.
  EXPECT_FALSE(hasSequenceHeader(concat({{0x30U}, kSequenceHeaderObuWithPayload})));
}

}  // namespace
}  // namespace livekit_ros2_bridge::video
