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

#include "video/h264_access_unit.hpp"

#include <cstddef>
#include <cstdint>

namespace livekit_ros2_bridge::video
{

namespace
{

constexpr std::uint8_t kNalUnitTypeMask = 0x1FU;
constexpr std::uint8_t kIdrSliceNalUnitType = 5U;

bool isStartCodeAt(const std::uint8_t * data, std::size_t offset)
{
  return data[offset] == 0x00U && data[offset + 1U] == 0x00U && data[offset + 2U] == 0x01U;
}

}  // namespace

bool containsIdrSlice(const std::uint8_t * data, std::size_t size)
{
  constexpr std::size_t kStartCodeSize = 3U;
  if (data == nullptr || size <= kStartCodeSize) {
    return false;
  }

  // Each candidate needs a full start code plus the NAL header byte after it.
  const std::size_t last_start_code_offset = size - kStartCodeSize - 1U;
  for (std::size_t offset = 0U; offset <= last_start_code_offset; ++offset) {
    if (!isStartCodeAt(data, offset)) {
      continue;
    }
    const std::uint8_t nal_header = data[offset + kStartCodeSize];
    if ((nal_header & kNalUnitTypeMask) == kIdrSliceNalUnitType) {
      return true;
    }
  }
  return false;
}

}  // namespace livekit_ros2_bridge::video
