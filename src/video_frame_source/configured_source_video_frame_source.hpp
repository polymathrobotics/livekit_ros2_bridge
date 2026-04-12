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

#pragma once

#include "video_frame_source/video_pipeline_frame_source.hpp"

namespace livekit_ros2_bridge
{

class ConfiguredSourceVideoFrameSource final : public VideoPipelineFrameSource
{
public:
  ConfiguredSourceVideoFrameSource(
    VideoStreamSpec spec,
    VideoFrameSink & frame_sink,
    VideoStreamLifecycleObserver & lifecycle_observer,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  void ensureRunning() override;
  void shutdown() override;

private:
  void startConfiguredSourcePipelineLocked();
  void resetSourceStateLocked() override;
  bool shouldRestartAfterFailure() const override;
  std::chrono::milliseconds restartDelayOnFailure() const override;
  void restartAfterFailureLocked() override;
};

}  // namespace livekit_ros2_bridge
