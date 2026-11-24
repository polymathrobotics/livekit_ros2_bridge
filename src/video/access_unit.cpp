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

#include "video/access_unit.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace livekit_ros2_bridge::video
{

namespace
{

struct NalUnitSyntax
{
  unsigned type_shift;
  std::uint8_t type_mask;
  std::uint8_t first_slice_type;
  std::uint8_t last_slice_type;
  std::uint8_t first_key_frame_type;
  std::uint8_t last_key_frame_type;
  std::uint64_t parameter_set_types;
};

constexpr std::uint64_t typeBit(unsigned nal_unit_type)
{
  return std::uint64_t{1} << nal_unit_type;
}

// Slices 1-5, key frames are IDR (5), SPS 7, PPS 8.
constexpr NalUnitSyntax kH264Syntax{0U, 0x1FU, 1U, 5U, 5U, 5U, typeBit(7U) | typeBit(8U)};
// Slices 0-31, key frames are the IRAP types 16-21 (BLA, IDR, CRA), VPS 32, SPS 33, PPS 34.
// An open GOP sends a CRA for every keyframe after the first. A decoder that starts at a CRA drops its RASL pictures,
// and the docs require no B-frames, so there are none.
constexpr NalUnitSyntax kH265Syntax{1U, 0x3FU, 0U, 31U, 16U, 21U, typeBit(32U) | typeBit(33U) | typeBit(34U)};

constexpr std::size_t kStartCodeSize = 3U;

bool isStartCodeAt(const std::uint8_t * data, std::size_t offset)
{
  return data[offset] == 0x00U && data[offset + 1U] == 0x00U && data[offset + 2U] == 0x01U;
}

// The offset of the first start code at or after `from`; it needs its NAL header byte inside `size` too.
std::optional<std::size_t> findStartCode(const std::uint8_t * data, std::size_t size, std::size_t from)
{
  for (std::size_t offset = from; offset + kStartCodeSize < size; ++offset) {
    if (isStartCodeAt(data, offset)) {
      return offset;
    }
  }
  return std::nullopt;
}

AccessUnitSummary summarizeAnnexBAccessUnit(const NalUnitSyntax & syntax, const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size <= kStartCodeSize) {
    return {};
  }

  std::uint64_t found_types = 0U;
  const auto has_parameter_sets = [&]() {
    return (found_types & syntax.parameter_set_types) == syntax.parameter_set_types;
  };

  for (std::optional<std::size_t> offset = findStartCode(data, size, 0U); offset.has_value();
       offset = findStartCode(data, size, *offset + 1U))
  {
    const auto nal_unit_type =
      static_cast<std::uint8_t>((data[*offset + kStartCodeSize] >> syntax.type_shift) & syntax.type_mask);
    if (nal_unit_type >= syntax.first_slice_type && nal_unit_type <= syntax.last_slice_type) {
      const bool is_key_frame =
        nal_unit_type >= syntax.first_key_frame_type && nal_unit_type <= syntax.last_key_frame_type;
      return {is_key_frame, has_parameter_sets()};
    }
    found_types |= typeBit(nal_unit_type);
  }
  return {false, has_parameter_sets()};
}

// Advances `offset` past a leb128; null if it isn't terminated within `size` or 8 bytes.
std::optional<std::uint64_t> readLeb128(const std::uint8_t * data, std::size_t size, std::size_t & offset)
{
  constexpr unsigned kMaxBytes = 8U;
  std::uint64_t value = 0U;
  for (unsigned byte_index = 0U; byte_index < kMaxBytes && offset < size; ++byte_index) {
    const std::uint8_t byte = data[offset];
    ++offset;
    value |= static_cast<std::uint64_t>(byte & 0x7FU) << (7U * byte_index);
    if ((byte & 0x80U) == 0U) {
      return value;
    }
  }
  return std::nullopt;
}

struct Av1Obu
{
  std::uint8_t type;
  std::size_t payload_offset;
  std::size_t payload_size;
};

// Reads the OBU at `offset` and moves `offset` past it; null if it is malformed or out of bytes.
// A live stream's OBUs all carry a size field, so one without it counts as malformed.
std::optional<Av1Obu> readAv1Obu(const std::uint8_t * data, std::size_t size, std::size_t & offset)
{
  constexpr std::uint8_t kHasExtensionBit = 0x04U;
  constexpr std::uint8_t kHasSizeFieldBit = 0x02U;
  if (offset >= size) {
    return std::nullopt;
  }

  const std::uint8_t header = data[offset];
  if ((header & kHasSizeFieldBit) == 0U) {
    return std::nullopt;
  }
  offset += (header & kHasExtensionBit) != 0U ? 2U : 1U;

  // An extension byte past the end leaves `offset` beyond `size`, which readLeb128 rejects.
  const std::optional<std::uint64_t> obu_size = readLeb128(data, size, offset);
  if (!obu_size.has_value() || *obu_size > size - offset) {
    return std::nullopt;
  }
  const Av1Obu obu{static_cast<std::uint8_t>((header >> 3U) & 0x0FU), offset, static_cast<std::size_t>(*obu_size)};
  offset += obu.payload_size;
  return obu;
}

// A frame shown later through show_existing_frame can't be resolved without the reference state, so
// it counts as a delta frame; it is rare.
bool isShownAv1KeyFrame(std::uint8_t first_frame_header_byte)
{
  constexpr std::uint8_t kShowExistingFrameBit = 0x80U;
  constexpr std::uint8_t kFrameTypeMask = 0x60U;
  constexpr std::uint8_t kShowFrameBit = 0x10U;
  // KEY_FRAME is frame_type 0.
  return (first_frame_header_byte & kShowExistingFrameBit) == 0U && (first_frame_header_byte & kFrameTypeMask) == 0U &&
         (first_frame_header_byte & kShowFrameBit) != 0U;
}

}  // namespace

AccessUnitSummary summarizeH264AccessUnit(const std::uint8_t * data, std::size_t size)
{
  return summarizeAnnexBAccessUnit(kH264Syntax, data, size);
}

AccessUnitSummary summarizeH265AccessUnit(const std::uint8_t * data, std::size_t size)
{
  return summarizeAnnexBAccessUnit(kH265Syntax, data, size);
}

AccessUnitSummary summarizeAv1TemporalUnit(const std::uint8_t * data, std::size_t size)
{
  constexpr std::uint8_t kSequenceHeaderObuType = 1U;
  constexpr std::uint8_t kFrameHeaderObuType = 3U;
  constexpr std::uint8_t kFrameObuType = 6U;
  if (data == nullptr) {
    return {};
  }

  bool has_sequence_header = false;
  std::size_t offset = 0U;
  for (std::optional<Av1Obu> obu = readAv1Obu(data, size, offset); obu.has_value();
       obu = readAv1Obu(data, size, offset))
  {
    if (obu->type == kSequenceHeaderObuType) {
      has_sequence_header = true;
    }
    if (obu->type == kFrameHeaderObuType || obu->type == kFrameObuType) {
      const bool is_key_frame = obu->payload_size > 0U && isShownAv1KeyFrame(data[obu->payload_offset]);
      return {is_key_frame, has_sequence_header};
    }
  }
  return {false, has_sequence_header};
}

std::size_t h264AccessUnitDelimiterSize(const std::uint8_t * data, std::size_t size)
{
  constexpr std::uint8_t kAccessUnitDelimiterType = 9U;
  if (data == nullptr) {
    return 0U;
  }

  const std::optional<std::size_t> first_start_code = findStartCode(data, size, 0U);
  if (!first_start_code.has_value()) {
    return 0U;
  }
  const std::size_t header_offset = *first_start_code + kStartCodeSize;
  if ((data[header_offset] & kH264Syntax.type_mask) != kAccessUnitDelimiterType) {
    return 0U;
  }

  // The next unit keeps its own start code, including the zero byte of a four-byte one.
  const std::optional<std::size_t> next_start_code = findStartCode(data, size, header_offset + 1U);
  if (!next_start_code.has_value()) {
    return 0U;
  }
  return data[*next_start_code - 1U] == 0x00U ? *next_start_code - 1U : *next_start_code;
}

}  // namespace livekit_ros2_bridge::video
