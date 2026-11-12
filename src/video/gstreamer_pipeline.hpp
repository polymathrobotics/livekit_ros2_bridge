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

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "video/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::video
{

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

enum class PipelineOutput
{
  // Raw I420 frames for LiveKit to encode.
  RawI420,
  // Annex-B H.264 access units forwarded to LiveKit unchanged (passthrough).
  EncodedH264,
};

struct PipelineCallbacks
{
  std::function<bool()> is_shutdown;
  std::function<void(const livekit::VideoFrame & frame, std::int64_t timestamp_us)> on_frame;
  std::function<void(const std::string & error)> on_unpack_failed;
  std::function<void(const std::string & error)> on_capture_failed;
  std::function<void(const std::string & reason)> on_failed;
  // EncodedH264 only.
  std::function<void(livekit::EncodedVideoSource::Frame frame)> on_encoded_frame;
  // EncodedH264 only: the bridge queue filled up because frames are consumed too slowly.
  std::function<void()> on_fell_behind;
  // EncodedH264 only: on_fell_behind threw.
  std::function<void(const std::string & error)> on_fell_behind_failed;
  // EncodedH264 only, instead of on_failed when set: the source's output did not
  // link or negotiate with the H.264 parser. offered_caps may be empty.
  std::function<void(const std::string & reason, const std::string & offered_caps)> on_caps_mismatch;
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

  void start(const std::string & description, bool require_appsrc, PipelineOutput output = PipelineOutput::RawI420);
  void stop();

  // Asks the source for a keyframe (upstream force-key-unit); a no-op without a pipeline. Call it
  // only from a pipeline callback, since start() resets the callback gate. The event runs through
  // the source's elements on the calling thread, so one that takes its stream lock could block.
  void requestKeyFrame();

private:
  bool beginCallback();
  void endCallback();
  void resumeCallbacks();
  void stopCallbacksAndWait();

  // Borrowed: callbacks_ lives as long as this pipeline.
  const PipelineCallbacks & callbacks() const;

  GstFlowReturn onSample(GstAppSink * sink);
  void onBusMessage(GstMessage * message);
  void onQueueOverrun();

  static constexpr std::size_t kCallbacksStopped = ~(~std::size_t{0} >> 1U);
  static constexpr std::size_t kCallbackCountMask = ~kCallbacksStopped;

  std::unique_ptr<PipelineCallbacks> callbacks_;
  std::atomic<const PipelineCallbacks *> callbacks_ptr_{nullptr};
  std::atomic<std::size_t> callback_state_{0};
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
  PipelineOutput output_ = PipelineOutput::RawI420;
  // EncodedH264 only: the bridge queue, held from start() so stop() disconnects
  // overrun_handler_ from the element it was connected to.
  GstElementPtr bridge_queue_;
  gulong overrun_handler_ = 0U;
  // Owned reference to the appsink's sink pad, released by stop(); atomic so
  // requestKeyFrame() can run on a pipeline callback's thread.
  std::atomic<GstPad *> key_frame_pad_{nullptr};
};

}  // namespace livekit_ros2_bridge::video
