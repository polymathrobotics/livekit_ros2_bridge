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
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

// Configured sources can restart immediately, but a small backoff avoids tight
// loops on a broken static launch string.
constexpr auto kRestartDelay = std::chrono::milliseconds(250);

}  // namespace

ConfiguredSourceVideoFrameSource::ConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: VideoPipelineFrameSource(
    spec,
    frame_sink,
    lifecycle_observer,
    std::move(profiler),
    VideoPipelineFrameSource::RestartConfig{
      buildFrameSourcePipelineDescription(spec.ingress_fragment, spec.transform_fragment),
      false,
      kRestartDelay,
    })
{}

void ConfiguredSourceVideoFrameSource::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (pipeline_ != nullptr) {
    return;
  }

  const auto & restart_config = restart_config_.value();
  startPipelineLocked(restart_config.pipeline_description, restart_config.require_appsrc);
}

void ConfiguredSourceVideoFrameSource::shutdown()
{
  PipelineHandles handles;
  bool restart_pending = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    restart_pending = recovery_pending_;
    is_shutdown_ = true;
    // Drop the internal handles while holding mutex_ so any in-flight
    // callbacks observe the terminal shutdown state before GStreamer teardown
    // removes callbacks and transitions the pipeline to NULL.
    handles = takePipelineLocked();
  }

  if (!restart_pending && handles.pipeline == nullptr && handles.appsrc == nullptr && handles.appsink == nullptr) {
    return;
  }

  if (handles.pipeline != nullptr || restart_pending) {
    LogEvent event(kLogger, "video_stream_source_shutdown");
    event.field("stream_key", spec_.stream_key);
    if (restart_pending) {
      event.field("restart_pending", true);
    }
    event.info();
  }

  teardown(handles.pipeline, handles.appsrc, handles.appsink);
}

}  // namespace livekit_ros2_bridge
