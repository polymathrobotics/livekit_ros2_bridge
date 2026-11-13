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

// How one codec's NAL header byte names the NAL unit types the scan needs.
struct NalUnitSyntax
{
  unsigned type_shift;
  std::uint8_t type_mask;
  std::uint8_t first_slice_type;
  std::uint8_t last_slice_type;
  std::uint8_t first_idr_type;
  std::uint8_t last_idr_type;
  // One bit per parameter set type a keyframe needs.
  std::uint64_t parameter_set_types;
};

constexpr std::uint64_t typeBit(unsigned nal_unit_type)
{
  return std::uint64_t{1} << nal_unit_type;
}

// Slices 1-5, IDR 5, SPS 7, PPS 8.
constexpr NalUnitSyntax kH264Syntax{0U, 0x1FU, 1U, 5U, 5U, 5U, typeBit(7U) | typeBit(8U)};
// Slices 0-31, IDR_W_RADL 19 and IDR_N_LP 20, VPS 32, SPS 33, PPS 34.
constexpr NalUnitSyntax kH265Syntax{1U, 0x3FU, 0U, 31U, 19U, 20U, typeBit(32U) | typeBit(33U) | typeBit(34U)};

bool isStartCodeAt(const std::uint8_t * data, std::size_t offset)
{
  return data[offset] == 0x00U && data[offset + 1U] == 0x00U && data[offset + 2U] == 0x01U;
}

AccessUnitSummary summarizeAnnexBAccessUnit(const NalUnitSyntax & syntax, const std::uint8_t * data, std::size_t size)
{
  constexpr std::size_t kStartCodeSize = 3U;
  if (data == nullptr || size <= kStartCodeSize) {
    return {};
  }

  std::uint64_t found_types = 0U;
  const auto has_parameter_sets = [&]() {
    return (found_types & syntax.parameter_set_types) == syntax.parameter_set_types;
  };

  // Each candidate needs a full start code plus the first NAL header byte after it.
  const std::size_t last_start_code_offset = size - kStartCodeSize - 1U;
  for (std::size_t offset = 0U; offset <= last_start_code_offset; ++offset) {
    if (!isStartCodeAt(data, offset)) {
      continue;
    }
    const auto nal_unit_type =
      static_cast<std::uint8_t>((data[offset + kStartCodeSize] >> syntax.type_shift) & syntax.type_mask);
    if (nal_unit_type >= syntax.first_slice_type && nal_unit_type <= syntax.last_slice_type) {
      const bool is_idr = nal_unit_type >= syntax.first_idr_type && nal_unit_type <= syntax.last_idr_type;
      return {is_idr, has_parameter_sets()};
    }
    found_types |= typeBit(nal_unit_type);
  }
  return {false, has_parameter_sets()};
}

// Reads the leb128 at `offset` and moves past it; null when it runs past `size` or past 8 bytes.
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

}  // namespace

AccessUnitSummary summarizeH264AccessUnit(const std::uint8_t * data, std::size_t size)
{
  return summarizeAnnexBAccessUnit(kH264Syntax, data, size);
}

AccessUnitSummary summarizeH265AccessUnit(const std::uint8_t * data, std::size_t size)
{
  return summarizeAnnexBAccessUnit(kH265Syntax, data, size);
}

bool hasAv1SequenceHeader(const std::uint8_t * data, std::size_t size)
{
  constexpr std::uint8_t kSequenceHeaderObuType = 1U;
  constexpr std::uint8_t kHasExtensionBit = 0x04U;
  constexpr std::uint8_t kHasSizeFieldBit = 0x02U;
  if (data == nullptr) {
    return false;
  }

  std::size_t offset = 0U;
  while (offset < size) {
    const std::uint8_t header = data[offset];
    const auto obu_type = static_cast<std::uint8_t>((header >> 3U) & 0x0FU);
    offset += (header & kHasExtensionBit) != 0U ? 2U : 1U;
    if (offset > size) {
      return false;
    }
    // Without a size field the OBU runs to the end of the temporal unit.
    if ((header & kHasSizeFieldBit) == 0U) {
      return obu_type == kSequenceHeaderObuType && offset < size;
    }

    const std::optional<std::uint64_t> obu_size = readLeb128(data, size, offset);
    if (!obu_size.has_value() || *obu_size > size - offset) {
      return false;
    }
    if (obu_type == kSequenceHeaderObuType) {
      return true;
    }
    offset += static_cast<std::size_t>(*obu_size);
  }
  return false;
}

}  // namespace livekit_ros2_bridge::video
