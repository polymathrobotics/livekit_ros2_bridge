// Copyright (c) 2025-present Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "video_frame_source/configured_source_video_frame_source.hpp"

#include <chrono>
#include <utility>

namespace livekit_ros2_bridge
{

namespace
{

// Configured sources can restart immediately, but a small backoff avoids tight
// loops on a broken static launch string.
constexpr auto kRestartDelay = std::chrono::milliseconds(250);

}  // namespace

ConfiguredSourceVideoFrameSource::ConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: VideoPipelineFrameSource(
    spec,
    sink,
    observer,
    std::move(profiler),
    VideoPipelineFrameSource::RestartConfig{
      buildVideoPipelineDescription(spec.ingress_fragment, spec.transform_fragment),
      false,
      kRestartDelay,
    })
{}

}  // namespace livekit_ros2_bridge
