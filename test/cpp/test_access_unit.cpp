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

#include <cstddef>
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
  EXPECT_FALSE(containsIdr(kLongStartCode));
  // A start code alone; the IDR header byte just past `size` must not be read.
  const Bytes start_code_then_idr = concat({kShortStartCode, {kIdrSlice}});
  EXPECT_FALSE(summarizeH264AccessUnit(start_code_then_idr.data(), kShortStartCode.size()).is_key_frame);
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
const Bytes kH265TrailingPictureNal{0x02U, 0x01U, 0xD0U, 0x08U};

constexpr std::uint8_t kFirstH265SliceType = 0U;
constexpr std::uint8_t kLastH265SliceType = 31U;
// BLA_W_LP, BLA_W_RADL, BLA_N_LP, IDR_W_RADL, IDR_N_LP and CRA_NUT; 22 and 23 are reserved.
constexpr std::uint8_t kFirstH265RandomAccessType = 16U;
constexpr std::uint8_t kLastH265RandomAccessType = 21U;

Bytes h265SliceOfType(std::uint8_t nal_unit_type)
{
  return {static_cast<std::uint8_t>(nal_unit_type << 1U), 0x01U, 0xAFU, 0x08U};
}

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

// libwebrtc's depacketizer marks the same types as key frames, and x265's open GOP sends a CRA for each.
TEST(H265AccessUnitTest, TreatsEveryRandomAccessPictureAsAKeyFrameWithItsParameterSets)
{
  for (std::uint8_t type = kFirstH265SliceType; type <= kLastH265SliceType; ++type) {
    SCOPED_TRACE(static_cast<int>(type));
    const AccessUnitSummary summary = summarizeH265(h265AccessUnitWithParameterSets(h265SliceOfType(type)));
    const bool is_random_access = type >= kFirstH265RandomAccessType && type <= kLastH265RandomAccessType;
    EXPECT_EQ(summary.is_key_frame, is_random_access);
    EXPECT_TRUE(summary.has_parameter_sets);

    // The first slice decides, whatever follows it.
    const AccessUnitSummary first_slice_decides =
      summarizeH265(concat({kLongStartCode, h265SliceOfType(type), kShortStartCode, kH265IdrWithLeadingPicturesNal}));
    EXPECT_EQ(first_slice_decides.is_key_frame, is_random_access);
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
}

// OBU header: (type << 3) | 0x02 when it has a size field, then a leb128 size.
constexpr std::uint8_t kSequenceHeaderObu = 0x0AU;
constexpr std::uint8_t kFrameHeaderObuType = 3U;
constexpr std::uint8_t kFrameObuType = 6U;
constexpr std::uint8_t kFrameObu = 0x32U;
constexpr std::uint8_t kMetadataObu = 0x2AU;
constexpr std::uint8_t kHasExtensionBit = 0x04U;
const Bytes kTemporalDelimiterObu{0x12U, 0x00U};
const Bytes kSequenceHeaderObuWithPayload{kSequenceHeaderObu, 0x03U, 0x00U, 0x00U, 0x00U};
const Bytes kFrameObuWithPayload{kFrameObu, 0x02U, 0x10U, 0x00U};

// The frame header opens with show_existing_frame (1 bit), frame_type (2) and show_frame (1).
constexpr std::uint8_t kShownKeyFrameBits = 0x10U;
constexpr std::uint8_t kHiddenKeyFrameBits = 0x00U;
constexpr std::uint8_t kShownInterFrameBits = 0x30U;
constexpr std::uint8_t kShownIntraOnlyFrameBits = 0x50U;
constexpr std::uint8_t kShownSwitchFrameBits = 0x70U;
// Shows a stored frame, whatever the bits after the first look like.
constexpr std::uint8_t kShowExistingFrameBits = 0x90U;

Bytes av1Obu(std::uint8_t obu_type, const Bytes & payload, bool with_extension = false)
{
  Bytes obu{static_cast<std::uint8_t>((obu_type << 3U) | 0x02U | (with_extension ? kHasExtensionBit : 0U))};
  if (with_extension) {
    obu.push_back(0x00U);
  }
  obu.push_back(static_cast<std::uint8_t>(payload.size()));
  obu.insert(obu.end(), payload.begin(), payload.end());
  return obu;
}

Bytes av1FrameObu(std::uint8_t first_payload_byte)
{
  return av1Obu(kFrameObuType, {first_payload_byte, 0x00U});
}

AccessUnitSummary summarizeAv1(const Bytes & bytes)
{
  return summarizeAv1TemporalUnit(bytes.data(), bytes.size());
}

bool hasSequenceHeader(const Bytes & bytes)
{
  return summarizeAv1(bytes).has_parameter_sets;
}

TEST(Av1TemporalUnitTest, SkipsExtensionBytesAndMultiByteSizes)
{
  // A 128-byte metadata OBU with an extension byte; its size is the two-byte leb128 80 01.
  Bytes metadata_with_extension{kMetadataObu | kHasExtensionBit, 0x08U, 0x80U, 0x01U};
  metadata_with_extension.resize(metadata_with_extension.size() + 128U, 0x00U);

  EXPECT_TRUE(hasSequenceHeader(concat({metadata_with_extension, kSequenceHeaderObuWithPayload})));
}

// `bytes` is a valid temporal unit, so a parser that reads past `size` sees it and gives itself away.
void expectNothingReadPast(const Bytes & bytes, std::size_t size)
{
  const AccessUnitSummary whole = summarizeAv1(bytes);
  ASSERT_TRUE(whole.is_key_frame || whole.has_parameter_sets) << "the untruncated bytes are not a valid unit";

  const AccessUnitSummary truncated = summarizeAv1TemporalUnit(bytes.data(), size);
  EXPECT_FALSE(truncated.is_key_frame);
  EXPECT_FALSE(truncated.has_parameter_sets);
}

TEST(Av1TemporalUnitTest, StopsSafelyAtTruncatedOrMalformedObus)
{
  EXPECT_FALSE(summarizeAv1TemporalUnit(nullptr, 0U).is_key_frame);
  // The sequence header claims five bytes but one is left.
  expectNothingReadPast(concat({Bytes{kSequenceHeaderObu, 0x05U}, Bytes(5U, 0x00U)}), 3U);
  // The frame claims five bytes but one is left, so the key frame bits it holds are not trusted.
  expectNothingReadPast(concat({Bytes{kFrameObu, 0x05U, kShownKeyFrameBits}, Bytes(4U, 0x00U)}), 3U);
  // The size field is cut off.
  expectNothingReadPast({kFrameObu, 0x01U, kShownKeyFrameBits}, 1U);
  // The extension byte is cut off.
  expectNothingReadPast({kFrameObu | kHasExtensionBit, 0x00U, 0x01U, kShownKeyFrameBits}, 1U);
  // The frame has a size field of zero, so it has no header byte, and the byte after it is not its own.
  const Bytes empty_frame_then_key_frame_bits{kFrameObu, 0x00U, kShownKeyFrameBits};
  EXPECT_FALSE(summarizeAv1TemporalUnit(empty_frame_then_key_frame_bits.data(), 2U).is_key_frame);
  // An OBU without a size field is malformed, not read to the end.
  EXPECT_FALSE(summarizeAv1({0x30U, kShownKeyFrameBits, 0x00U}).is_key_frame);
}

// A shown key frame whose size field is `size_bytes`, a leb128 that means a payload of one byte.
AccessUnitSummary summarizeKeyFrameAfterSequenceHeader(const Bytes & size_bytes)
{
  return summarizeAv1(concat({kSequenceHeaderObuWithPayload, Bytes{kFrameObu}, size_bytes, Bytes{kShownKeyFrameBits}}));
}

TEST(Av1TemporalUnitTest, ReadsAnObuSizeOfAtMostEightBytes)
{
  // A leb128 may carry padding, so 1 can be written as 81 00.
  const AccessUnitSummary padded = summarizeKeyFrameAfterSequenceHeader({0x81U, 0x00U});
  EXPECT_TRUE(padded.is_key_frame);
  EXPECT_TRUE(padded.has_parameter_sets);

  // Eight bytes is the longest size that is read: 81, six 80s and a 00.
  const AccessUnitSummary eight_bytes =
    summarizeKeyFrameAfterSequenceHeader({0x81U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x00U});
  EXPECT_TRUE(eight_bytes.is_key_frame);
  EXPECT_TRUE(eight_bytes.has_parameter_sets);

  // A ninth byte is one too many, so the walk stops before the frame, but the sequence header still counts.
  const AccessUnitSummary nine_bytes =
    summarizeKeyFrameAfterSequenceHeader({0x81U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U, 0x00U});
  EXPECT_FALSE(nine_bytes.is_key_frame);
  EXPECT_TRUE(nine_bytes.has_parameter_sets);
}

TEST(Av1TemporalUnitTest, ReadsKeyFramesFromBothFrameObuTypes)
{
  for (const std::uint8_t obu_type : {kFrameHeaderObuType, kFrameObuType}) {
    SCOPED_TRACE(static_cast<int>(obu_type));
    const Bytes frame = av1Obu(obu_type, {kShownKeyFrameBits, 0x00U});

    const AccessUnitSummary with_sequence_header =
      summarizeAv1(concat({kTemporalDelimiterObu, kSequenceHeaderObuWithPayload, frame}));
    EXPECT_TRUE(with_sequence_header.is_key_frame);
    EXPECT_TRUE(with_sequence_header.has_parameter_sets);

    const AccessUnitSummary without_sequence_header = summarizeAv1(concat({kTemporalDelimiterObu, frame}));
    EXPECT_TRUE(without_sequence_header.is_key_frame);
    EXPECT_FALSE(without_sequence_header.has_parameter_sets);
  }
}

TEST(Av1TemporalUnitTest, RejectsFramesADecoderCannotStartFrom)
{
  for (const std::uint8_t frame_bits :
       {kShownInterFrameBits,
        kShownIntraOnlyFrameBits,
        kShownSwitchFrameBits,
        kHiddenKeyFrameBits,
        kShowExistingFrameBits})
  {
    SCOPED_TRACE(static_cast<int>(frame_bits));
    const AccessUnitSummary summary =
      summarizeAv1(concat({kTemporalDelimiterObu, kSequenceHeaderObuWithPayload, av1FrameObu(frame_bits)}));
    EXPECT_FALSE(summary.is_key_frame);
    EXPECT_TRUE(summary.has_parameter_sets);
  }
}

TEST(Av1TemporalUnitTest, DecidesAtTheFirstFrameAndSkipsOtherObus)
{
  const Bytes metadata_obu = av1Obu(5U, {kShownKeyFrameBits, 0x00U});
  const Bytes tile_group_obu = av1Obu(4U, {kShownKeyFrameBits, 0x00U});
  EXPECT_FALSE(summarizeAv1(concat({kTemporalDelimiterObu, metadata_obu, tile_group_obu})).is_key_frame);
  EXPECT_TRUE(
    summarizeAv1(concat({kTemporalDelimiterObu, metadata_obu, av1FrameObu(kShownKeyFrameBits), tile_group_obu}))
      .is_key_frame);
  EXPECT_TRUE(summarizeAv1(concat({av1FrameObu(kShownKeyFrameBits), av1FrameObu(kShownInterFrameBits)})).is_key_frame);
  EXPECT_FALSE(summarizeAv1(concat({av1FrameObu(kShownInterFrameBits), av1FrameObu(kShownKeyFrameBits)})).is_key_frame);
  // The walk stops at the frame, so a sequence header after it does not count.
  EXPECT_FALSE(hasSequenceHeader(concat({av1FrameObu(kShownKeyFrameBits), kSequenceHeaderObuWithPayload})));
}

TEST(Av1TemporalUnitTest, ReadsTheFrameHeaderPastAnExtensionByte)
{
  const Bytes & sequence_header = kSequenceHeaderObuWithPayload;
  EXPECT_TRUE(
    summarizeAv1(concat({sequence_header, av1Obu(kFrameObuType, {kShownKeyFrameBits, 0x00U}, true)})).is_key_frame);
  EXPECT_FALSE(
    summarizeAv1(concat({sequence_header, av1Obu(kFrameObuType, {kShownInterFrameBits, 0x00U}, true)})).is_key_frame);
}

}  // namespace
}  // namespace livekit_ros2_bridge::video
