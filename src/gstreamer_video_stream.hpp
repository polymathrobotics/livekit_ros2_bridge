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

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "gstreamer_pipeline.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class VideoTrackPublisher;

// Pipeline callbacks can run on GStreamer threads; mutex_ guards shutdown and
// restart state.
class GStreamerVideoStream final
{
public:
  GStreamerVideoStream(VideoStreamSpec spec, VideoTrackPublisher & publisher);
  ~GStreamerVideoStream();

  GStreamerVideoStream(const GStreamerVideoStream &) = delete;
  GStreamerVideoStream & operator=(const GStreamerVideoStream &) = delete;
  GStreamerVideoStream(GStreamerVideoStream &&) = delete;
  GStreamerVideoStream & operator=(GStreamerVideoStream &&) = delete;

  void start();
  void close();

private:
  // Coalesces repeated EOS/error messages while a restart is pending.
  void onPipelineFailure(const std::string & reason);
  std::string buildDescription() const;
  bool isShutdown() const;
  void restartLoop();
  void restart();

  VideoStreamSpec spec_;
  VideoTrackPublisher & publisher_;
  GStreamerPipeline pipeline_;
  mutable std::mutex mutex_;
  bool is_shutdown_ = false;
  bool restart_pending_ = false;
  std::condition_variable restart_condition_;
  std::thread restart_thread_;
};

}  // namespace livekit_ros2_bridge
