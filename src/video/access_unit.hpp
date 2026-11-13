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

#pragma once

#include <cstddef>
#include <cstdint>

namespace livekit_ros2_bridge::video
{

struct AccessUnitSummary
{
  // Holds a keyframe a WebRTC viewer can start from: for H.264 and H.265, an IDR picture.
  bool is_key_frame = false;
  // Holds the parameter sets (H.264 SPS and PPS, H.265 VPS, SPS and PPS, AV1 sequence header)
  // that a viewer starting at this frame needs to decode it.
  bool has_parameter_sets = false;
};

// Scan an Annex-B access unit up to its first coded slice, since a picture's slices share a type
// and its parameter sets precede them; every 00 00 01 counts as a start code. `data` may be null
// only when `size == 0`.
AccessUnitSummary summarizeH264AccessUnit(const std::uint8_t * data, std::size_t size);
AccessUnitSummary summarizeH265AccessUnit(const std::uint8_t * data, std::size_t size);

// Whether an AV1 temporal unit in the low-overhead OBU format holds a sequence header OBU. A
// malformed OBU ends the walk. `data` may be null only when `size == 0`.
bool hasAv1SequenceHeader(const std::uint8_t * data, std::size_t size);

}  // namespace livekit_ros2_bridge::video
