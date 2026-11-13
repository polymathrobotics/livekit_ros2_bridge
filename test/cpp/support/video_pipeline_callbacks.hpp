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

#include <cstdint>
#include <string>

#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "video/gstreamer_pipeline.hpp"

namespace livekit_ros2_bridge::test_support
{

// Video pipeline callbacks that do nothing, for tests to override the ones they observe.
inline video::PipelineCallbacks makeNoOpVideoPipelineCallbacks()
{
  video::PipelineCallbacks callbacks;
  callbacks.is_shutdown = []() { return false; };
  callbacks.on_frame = [](const livekit::VideoFrame &, std::int64_t) {};
  callbacks.on_unpack_failed = [](const std::string &) {};
  callbacks.on_capture_failed = [](const std::string &) {};
  callbacks.on_failed = [](const std::string &) {};
  callbacks.on_encoded_frame = [](livekit::EncodedVideoSource::Frame) {};
  callbacks.on_fell_behind = []() {};
  callbacks.on_caps_mismatch = [](const std::string &, const std::string &) {};
  callbacks.on_key_frame_without_parameter_sets = []() {};
  return callbacks;
}

}  // namespace livekit_ros2_bridge::test_support
