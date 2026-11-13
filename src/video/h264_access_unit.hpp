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
  // Holds an IDR slice (NAL type 5), the only frame a WebRTC viewer can start from.
  bool is_idr = false;
  // Holds an SPS (NAL type 7) and a PPS (NAL type 8) before its first slice; a viewer
  // starting at this access unit cannot decode it without them.
  bool has_parameter_sets = false;
};

// Scans an Annex-B H.264 access unit up to its first coded slice, since a picture's slices share
// a type and its parameter sets precede them; every 00 00 01 counts as a start code. `data` may
// be null only when `size == 0`.
AccessUnitSummary summarizeAccessUnit(const std::uint8_t * data, std::size_t size);

}  // namespace livekit_ros2_bridge::video
