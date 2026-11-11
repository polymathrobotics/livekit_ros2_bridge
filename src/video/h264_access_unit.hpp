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

// Returns true if the Annex-B H.264 access unit holds an IDR slice (NAL type 5).
// h264parse also marks non-IDR I-frames and recovery points as keyframes, but a
// WebRTC viewer can only start decoding from an IDR.
// All slices of one picture share the same NAL type, so the scan stops at the
// first coded slice (NAL type 1 or 5) and decides from it.
// Every 00 00 01 is treated as a start code (4-byte start codes end in one);
// emulation prevention keeps that sequence out of NAL payloads.
// `data` may be null only when `size == 0`.
bool containsIdrSlice(const std::uint8_t * data, std::size_t size);

}  // namespace livekit_ros2_bridge::video
