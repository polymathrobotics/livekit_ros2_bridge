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
#include <memory>
#include <stdexcept>
#include <utility>

#include "video_frame_source.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kConfiguredSourceRestartDelay = std::chrono::milliseconds(250);

}  // namespace

ConfiguredSourceVideoFrameSource::ConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: VideoPipelineFrameSource(std::move(spec), frame_sink, lifecycle_observer, std::move(profiler))
{}

void ConfiguredSourceVideoFrameSource::ensureRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (pipeline_ == nullptr) {
    startConfiguredSourcePipelineLocked();
  }
}

void ConfiguredSourceVideoFrameSource::shutdown()
{
  DetachedPipelineState detached_pipeline_state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    detached_pipeline_state = detachPipelineStateLocked();
  }

  teardownDetachedPipelineState(detached_pipeline_state);
}

void ConfiguredSourceVideoFrameSource::startConfiguredSourcePipelineLocked()
{
  startPipelineLocked(composeVideoPipeline(spec_.ingress_fragment, spec_.transform_fragment));
  playPipelineLocked();
}

void ConfiguredSourceVideoFrameSource::resetSourceStateLocked()
{}

bool ConfiguredSourceVideoFrameSource::shouldRestartAfterFailure() const
{
  return true;
}

std::chrono::milliseconds ConfiguredSourceVideoFrameSource::restartDelayOnFailure() const
{
  return kConfiguredSourceRestartDelay;
}

void ConfiguredSourceVideoFrameSource::restartAfterFailureLocked()
{
  startConfiguredSourcePipelineLocked();
}

std::shared_ptr<VideoFrameSource> makeConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
{
  return std::make_shared<ConfiguredSourceVideoFrameSource>(
    std::move(spec), frame_sink, lifecycle_observer, std::move(profiler));
}

}  // namespace livekit_ros2_bridge
