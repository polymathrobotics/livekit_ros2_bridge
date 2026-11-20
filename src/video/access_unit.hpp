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
  bool is_key_frame = false;
  // The parameter sets (AV1: sequence header) a viewer starting here cannot decode without.
  bool has_parameter_sets = false;
};

// Stops at the first coded slice: its type decides, and the parameter sets precede it.
AccessUnitSummary summarizeH264AccessUnit(const std::uint8_t * data, std::size_t size);
AccessUnitSummary summarizeH265AccessUnit(const std::uint8_t * data, std::size_t size);

// Stops at the first frame, whose header decides; the sequence header precedes it.
// Every OBU must carry a size field, and a malformed one ends the walk.
AccessUnitSummary summarizeAv1TemporalUnit(const std::uint8_t * data, std::size_t size);

}  // namespace livekit_ros2_bridge::video
