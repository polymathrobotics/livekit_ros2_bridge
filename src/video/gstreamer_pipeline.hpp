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

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "livekit/video_frame.h"
#include "video/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::video
{

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

struct PipelineCallbacks
{
  std::function<bool()> is_shutdown;
  std::function<void(const livekit::VideoFrame & frame, std::int64_t timestamp_us)> on_frame;
  std::function<void(const std::string & error)> on_unpack_failed;
  std::function<void(const std::string & error)> on_capture_failed;
  std::function<void(const std::string & reason)> on_failed;
};

class GStreamerPipeline final
{
public:
  explicit GStreamerPipeline(PipelineCallbacks callbacks);
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
  bool beginCallback();
  void endCallback();
  void resumeCallbacks();
  void stopCallbacksAndWait();

  PipelineCallbacks callbacks() const;

  GstFlowReturn onSample(GstAppSink * sink);
  void onBusMessage(GstMessage * message);

  static constexpr std::size_t kCallbacksStopped = ~(~std::size_t{0} >> 1U);
  static constexpr std::size_t kCallbackCountMask = ~kCallbacksStopped;

  std::unique_ptr<PipelineCallbacks> callbacks_;
  std::atomic<const PipelineCallbacks *> callbacks_ptr_{nullptr};
  std::atomic<std::size_t> callback_state_{0};
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
};

}  // namespace livekit_ros2_bridge::video
