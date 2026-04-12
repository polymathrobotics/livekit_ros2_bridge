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
#include <memory>
#include <mutex>
#include <string>

#include "utils/gstreamer_raii.hpp"
#include "video_frame_source.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

inline constexpr char kVideoAppSrcName[] = "bridge_video_src";

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

std::string composeVideoPipeline(const std::string & ingress_fragment, const std::string & transform_fragment);

class VideoPipelineFrameSource : public VideoFrameSource, public std::enable_shared_from_this<VideoPipelineFrameSource>
{
public:
  VideoPipelineFrameSource(
    VideoStreamSpec spec, VideoFrameSink & frame_sink, VideoStreamLifecycleObserver & lifecycle_observer);
  ~VideoPipelineFrameSource() override;

protected:
  struct DetachedPipelineState
  {
    GstElementPtr pipeline;
    GstAppSrcPtr appsrc;
    GstAppSinkPtr appsink;
  };

  static GstFlowReturn onNewSampleThunk(GstAppSink * sink, gpointer user_data);
  static GstBusSyncReply onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data);

  DetachedPipelineState detachPipelineStateLocked();
  static void teardownDetachedPipelineState(DetachedPipelineState & detached);

  void startPipelineLocked(const std::string & pipeline_description, bool expect_appsrc = false);
  void playPipelineLocked();
  void discardPipelineElementsLocked();
  void stopPipelineLocked();

  virtual void resetSourceStateLocked() = 0;
  virtual bool shouldRestartAfterFailure() const;
  virtual std::chrono::milliseconds restartDelayOnFailure() const;
  virtual void restartAfterFailureLocked();

  VideoStreamSpec spec_;
  VideoFrameSink & frame_sink_;
  VideoStreamLifecycleObserver & lifecycle_observer_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  bool failure_recovery_pending_ = false;
  bool first_sample_logged_ = false;
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
  GstAppSinkPtr appsink_;

private:
  GstFlowReturn onNewSample(GstAppSink * sink);
  void onBusMessage(GstMessage * message);
  void handlePipelineFailure(const std::string & reason);
  void recoverFromPipelineFailure();
};

}  // namespace livekit_ros2_bridge
