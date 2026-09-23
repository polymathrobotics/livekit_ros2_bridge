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

#include <atomic>
#include <mutex>
#include <string>

#include "audio/gstreamer_pipeline.hpp"
#include "audio/stream_spec.hpp"
#include "utils/pipeline_failure_handler.hpp"

namespace livekit_ros2_bridge::audio
{

class TrackPublisher;

// Pipeline callbacks can run on GStreamer threads; mutex_ guards shutdown and
// restart state.
class GStreamerStream final
{
public:
  GStreamerStream(StreamSpec spec, TrackPublisher & publisher);
  ~GStreamerStream();

  GStreamerStream(const GStreamerStream &) = delete;
  GStreamerStream & operator=(const GStreamerStream &) = delete;
  GStreamerStream(GStreamerStream &&) = delete;
  GStreamerStream & operator=(GStreamerStream &&) = delete;

  void start();
  void close();

private:
  // Coalesces repeated EOS/error messages while a restart is pending.
  void onPipelineFailure(const std::string & reason);
  void restartPipelineAfterFailure();
  void startPipelineLocked();

  StreamSpec spec_;
  TrackPublisher & publisher_;
  GStreamerPipeline pipeline_;
  mutable std::mutex mutex_;
  std::atomic<bool> is_shutdown_{false};
  utils::PipelineFailureHandler failure_handler_;
};

}  // namespace livekit_ros2_bridge::audio
