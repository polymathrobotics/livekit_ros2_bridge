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
#include <chrono>
#include <mutex>
#include <string>

#include "utils/event_throttle.hpp"
#include "utils/pipeline_failure_handler.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/stream_spec.hpp"

namespace livekit_ros2_bridge::video
{

class TrackPublisher;

// Pipeline callbacks can run on GStreamer threads; mutex_ serialises start, close and
// restart, and the failure path never takes it.
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
  PipelineCallbacks makePipelineCallbacks();
  // Coalesces repeated failures (EOS/error messages, a restart that throws) while a restart
  // is pending. caps_mismatch adds the passthrough H.264 format hint to the log.
  void onPipelineFailure(const std::string & reason, bool caps_mismatch = false, const std::string & offered_caps = {});
  void restartPipelineAfterFailure();
  void startPipelineLocked();

  static constexpr auto kLogThrottle = std::chrono::seconds(5);

  StreamSpec spec_;
  TrackPublisher & publisher_;
  GStreamerPipeline pipeline_;
  std::mutex mutex_;
  std::atomic<bool> is_shutdown_{false};
  // A source that stays broken fails every restart delay; log it once per interval. The mutex
  // guards only the throttle and is never held across GStreamer calls. Both are declared before
  // failure_handler_, so they outlive its worker, which uses them.
  std::mutex recovery_log_mutex_;
  EventThrottle recovery_log_throttle_{kLogThrottle};
  utils::PipelineFailureHandler failure_handler_;
};

}  // namespace livekit_ros2_bridge::video
