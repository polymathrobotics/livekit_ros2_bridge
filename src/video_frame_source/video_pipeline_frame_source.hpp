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

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "utils/gstreamer_raii.hpp"
#include "video_pipeline_description.hpp"
#include "video_stream_spec.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

class VideoPipelineFrameSource : public VideoFrameSource, public std::enable_shared_from_this<VideoPipelineFrameSource>
{
public:
  // Configures optional self-recovery for sources whose pipeline can be
  // recreated from a stable launch description after EOS or ERROR.
  // TODO: let fixed-pipeline sources supply restart descriptions on demand
  // without going through this generic config bundle.
  struct RestartConfig
  {
    // Requires the named appsrc and captures its handle so ROS-backed
    // subclasses can resume pushing into a restarted pipeline.
    bool require_appsrc = false;
    // Optional backoff to avoid tight restart loops for hot-failing pipelines.
    std::chrono::milliseconds restart_delay = std::chrono::milliseconds(0);
  };

  VideoPipelineFrameSource(
    VideoStreamSpec spec,
    VideoFrameSink & sink,
    VideoStreamLifecycleObserver & observer,
    std::optional<RestartConfig> restart_config = std::nullopt);
  ~VideoPipelineFrameSource() override;

  void close() override;

  // Starts the fixed pipeline derived from spec_. Concrete sources call this
  // once after construction when they should own an active pipeline immediately.
  void activateFixedPipeline();

protected:
  // Moves the live GStreamer handles out of member state while mutex_ is held
  // so callers can choose when teardown runs without leaving dangling members.
  struct PipelineHandles
  {
    GstElementPtr pipeline;
    GstAppSrcPtr appsrc;
    GstAppSinkPtr appsink;
  };

  static GstFlowReturn onSampleThunk(GstAppSink * sink, gpointer user_data);
  static GstBusSyncReply onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data);

  // Disconnects callbacks before forcing the pipeline to NULL so GStreamer
  // does not call back into this object after ownership has been detached.
  static void teardown(GstElementPtr & pipeline, GstAppSrcPtr & appsrc, GstAppSinkPtr & appsink);

  VideoStreamSpec spec_;
  VideoFrameSink & sink_;
  VideoStreamLifecycleObserver & observer_;
  const std::optional<RestartConfig> restart_config_;
  // Guards lifecycle flags and GStreamer handle ownership across activation,
  // close(), appsink callbacks, bus callbacks, and recovery.
  std::mutex mutex_;
  bool is_shutdown_ = false;
  // Coalesces repeated EOS/ERROR notifications so only one async recovery path
  // tears down and rebuilds the current pipeline at a time.
  bool recovery_pending_ = false;
  // Owned handles for the currently installed pipeline. They are moved out
  // under mutex_ before teardown so subsequent callbacks see detached members.
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
  GstAppSinkPtr appsink_;

  // Resets subclass-specific bookkeeping and atomically transfers the current
  // pipeline handles out of the object. Caller must hold mutex_.
  [[nodiscard]] PipelineHandles takePipelineLocked();
  // Marks shutdown terminal, wakes any delayed recovery, and transfers the
  // owned worker thread so callers can join without holding mutex_.
  [[nodiscard]] std::thread beginShutdownLocked();
  static void joinRecoveryThread(std::thread & recovery_thread);
  // Caller must hold mutex_. Parses `description`, validates the named
  // app endpoints, installs callbacks, and transitions the pipeline to PLAYING.
  void startPipelineLocked(const std::string & description, bool require_appsrc = false);

  virtual void resetLocked();

private:
  GstFlowReturn onSample(GstAppSink * sink);
  void onBusMessage(GstMessage * message);
  void handleFailure(const std::string & reason);
  void ensureRecoveryThreadLocked();
  void recoveryLoop();
  void recoverAfterFailure();
  std::string fixedDescription() const;

  std::condition_variable recovery_condition_;
  std::thread recovery_thread_;
};

std::shared_ptr<VideoFrameSource> makeOtherVideoFrameSource(
  VideoStreamSpec spec, VideoFrameSink & sink, VideoStreamLifecycleObserver & observer);

}  // namespace livekit_ros2_bridge
