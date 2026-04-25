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

#include <cstdint>
#include <functional>
#include <string>

#include "livekit/video_frame.h"
#include "utils/gstreamer_raii.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

struct GStreamerPipelineCallbacks
{
  std::function<bool()> is_shutdown;
  std::function<void(const livekit::VideoFrame & frame, std::int64_t timestamp_us)> on_frame;
  std::function<void(const std::string & error)> on_unpack_failed;
  std::function<void(const std::string & error)> on_capture_failed;
  std::function<void(const std::string & reason)> on_pipeline_failed;
};

class GStreamerPipeline final
{
public:
  GStreamerPipeline(VideoStreamSpec spec, GStreamerPipelineCallbacks callbacks);
  ~GStreamerPipeline();

  GStreamerPipeline(const GStreamerPipeline &) = delete;
  GStreamerPipeline & operator=(const GStreamerPipeline &) = delete;
  GStreamerPipeline(GStreamerPipeline &&) = delete;
  GStreamerPipeline & operator=(GStreamerPipeline &&) = delete;

  bool isActive() const noexcept;
  // Borrowed pointer; non-null after start(..., require_appsrc=true) until stop() or next start().
  GstAppSrc * appsrc() const noexcept;

  void start(const std::string & description, bool require_appsrc);
  void stop();

private:
  static GstFlowReturn onSampleThunk(GstAppSink * sink, gpointer user_data);
  static GstBusSyncReply onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data);

  GstFlowReturn onSample(GstAppSink * sink);
  void onBusMessage(GstMessage * message);

  VideoStreamSpec spec_;
  GStreamerPipelineCallbacks callbacks_;
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
};

}  // namespace livekit_ros2_bridge
