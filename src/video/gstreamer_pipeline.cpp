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

#include "video/gstreamer_pipeline.hpp"

#include <gst/video/video.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include "livekit/video_source.h"
#include "utils/scope_exit.hpp"
#include "video/pipeline_description.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

void validateI420Plane(const GstVideoFrame * frame, guint component, const livekit::VideoPlaneInfo & plane)
{
  if (plane.stride == 0U || plane.size % plane.stride != 0U) {
    throw std::runtime_error("LiveKit I420 plane layout is invalid.");
  }
  const std::size_t height = static_cast<std::size_t>(plane.size / plane.stride);
  if (
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_WIDTH(frame, component)) != plane.stride ||
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_HEIGHT(frame, component)) != height)
  {
    throw std::runtime_error("Unexpected I420 plane dimensions from GStreamer.");
  }
}

void copyI420Plane(const GstVideoFrame * frame, guint component, const livekit::VideoPlaneInfo & plane)
{
  const auto * src = static_cast<const std::uint8_t *>(GST_VIDEO_FRAME_COMP_DATA(frame, component));
  const int src_stride = GST_VIDEO_FRAME_COMP_STRIDE(frame, component);
  const int width = GST_VIDEO_FRAME_COMP_WIDTH(frame, component);
  const int height = GST_VIDEO_FRAME_COMP_HEIGHT(frame, component);
  if (src == nullptr) {
    throw std::runtime_error("I420 frame plane data is unavailable.");
  }
  if (src_stride < 0 || width <= 0 || height <= 0) {
    throw std::runtime_error("I420 frame plane layout is invalid.");
  }
  if (plane.data_ptr == 0U) {
    throw std::runtime_error("LiveKit I420 plane data is unavailable.");
  }
  auto * dst = reinterpret_cast<std::uint8_t *>(plane.data_ptr);
  const std::size_t dst_stride = plane.stride;
  if (static_cast<std::size_t>(width) > dst_stride || src_stride < width) {
    throw std::runtime_error("I420 frame plane stride is unsupported.");
  }

  const std::size_t src_stride_bytes = static_cast<std::size_t>(src_stride);
  const std::size_t width_bytes = static_cast<std::size_t>(width);
  for (int row = 0; row < height; ++row) {
    std::memcpy(
      dst + static_cast<std::size_t>(row) * dst_stride,
      src + static_cast<std::size_t>(row) * src_stride_bytes,
      width_bytes);
  }
}

livekit::VideoFrame unpackI420Frame(GstSample * sample)
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (caps == nullptr || buffer == nullptr) {
    throw std::runtime_error("GStreamer sample is missing caps or buffer.");
  }

  GstVideoInfo info;
  if (!gst_video_info_from_caps(&info, caps)) {
    throw std::runtime_error("Failed to parse GStreamer video caps.");
  }
  if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_I420) {
    throw std::runtime_error("Video pipeline did not output I420 frames.");
  }

  const int width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&info));
  const int height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info));
  if (width <= 0 || height <= 0) {
    throw std::runtime_error("I420 sample dimensions are invalid.");
  }

  GstVideoFrameMap mapping(info, *buffer, GST_MAP_READ);
  if (!mapping.is_valid()) {
    throw std::runtime_error("Failed to map GStreamer video frame.");
  }

  // LiveKit wants tightly packed I420; copy padded GStreamer planes row-by-row.
  const auto * src_frame = mapping.get();
  auto frame = livekit::VideoFrame::create(width, height, livekit::VideoBufferType::I420);
  const auto planes = frame.planeInfos();
  if (planes.size() != 3U) {
    throw std::runtime_error("LiveKit I420 plane layout is invalid.");
  }
  validateI420Plane(src_frame, 0, planes[0]);
  validateI420Plane(src_frame, 1, planes[1]);
  validateI420Plane(src_frame, 2, planes[2]);
  copyI420Plane(src_frame, 0, planes[0]);
  copyI420Plane(src_frame, 1, planes[1]);
  copyI420Plane(src_frame, 2, planes[2]);
  return frame;
}

}  // namespace

GStreamerPipeline::GStreamerPipeline(PipelineCallbacks callbacks)
{
  callbacks_ = std::make_unique<PipelineCallbacks>(std::move(callbacks));
  callbacks_ptr_.store(callbacks_.get(), std::memory_order_release);
}

GStreamerPipeline::~GStreamerPipeline()
{
  stop();
}

bool GStreamerPipeline::isActive() const noexcept
{
  return pipeline_ != nullptr;
}

GstAppSrc * GStreamerPipeline::appsrc() const noexcept
{
  return appsrc_.get();
}

PipelineCallbacks GStreamerPipeline::callbacks() const
{
  const auto * callbacks = callbacks_ptr_.load(std::memory_order_acquire);
  return callbacks == nullptr ? PipelineCallbacks{} : *callbacks;
}

// GStreamer may enter raw C callbacks from streaming/bus threads while stop()
// clears callback pointers, so this tight atomic gate intentionally stays local.
bool GStreamerPipeline::beginCallback()
{
  std::size_t state = callback_state_.load(std::memory_order_acquire);
  while ((state & kCallbacksStopped) == 0U) {
    if (callback_state_.compare_exchange_weak(state, state + 1U, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      return true;
    }
  }
  return false;
}

void GStreamerPipeline::endCallback()
{
  (void)callback_state_.fetch_sub(1U, std::memory_order_acq_rel);
}

void GStreamerPipeline::resumeCallbacks()
{
  callback_state_.store(0U, std::memory_order_release);
}

void GStreamerPipeline::stopCallbacksAndWait()
{
  std::size_t state = callback_state_.fetch_or(kCallbacksStopped, std::memory_order_acq_rel) | kCallbacksStopped;
  while ((state & kCallbackCountMask) != 0U) {
    std::this_thread::yield();
    state = callback_state_.load(std::memory_order_acquire);
  }
}

void GStreamerPipeline::start(const std::string & description, bool require_appsrc)
{
  ensureGStreamerInitialized();
  stop();
  resumeCallbacks();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  if (pipeline == nullptr) {
    const std::string message = error != nullptr ? error->message : "gst_parse_launch returned null";
    throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
  }
  if (!GST_IS_BIN(pipeline.get())) {
    throw std::runtime_error("Video pipeline must resolve to a GstBin.");
  }

  GstElementPtr sink_element(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeAppSinkName));
  if (sink_element == nullptr) {
    throw std::runtime_error("Video pipeline did not create the expected appsink.");
  }
  if (!GST_IS_APP_SINK(sink_element.get())) {
    throw std::runtime_error(std::string("Video pipeline named ") + kBridgeAppSinkName + " must be a GstAppSink.");
  }

  GstAppSrcPtr appsrc;
  if (require_appsrc) {
    GstElementPtr appsrc_element(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeAppSrcName));
    if (appsrc_element == nullptr) {
      throw std::runtime_error("Video pipeline did not create the expected appsrc.");
    }
    if (!GST_IS_APP_SRC(appsrc_element.get())) {
      throw std::runtime_error(std::string("Video pipeline named ") + kBridgeAppSrcName + " must be a GstAppSrc.");
    }
    appsrc = GstAppSrcPtr(GST_APP_SRC(appsrc_element.release()));
  }

  GstAppSinkPtr appsink(GST_APP_SINK(sink_element.release()));
  // stop() clears raw this callbacks before releasing the bin.
  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = [](GstAppSink * sink, gpointer user_data) -> GstFlowReturn {
    return static_cast<GStreamerPipeline *>(user_data)->onSample(sink);
  };
  gst_app_sink_set_callbacks(appsink.get(), &callbacks, this, nullptr);

  GstBusPtr bus(gst_element_get_bus(pipeline.get()));
  gst_bus_set_sync_handler(
    bus.get(),
    [](GstBus *, GstMessage * message, gpointer user_data) -> GstBusSyncReply {
      static_cast<GStreamerPipeline *>(user_data)->onBusMessage(message);
      return GST_BUS_PASS;
    },
    this,
    nullptr);

  pipeline_ = std::move(pipeline);
  appsrc_ = std::move(appsrc);

  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (result == GST_STATE_CHANGE_FAILURE) {
    stop();
    throw std::runtime_error("Failed to set video pipeline to PLAYING.");
  }
}

void GStreamerPipeline::stop()
{
  if (pipeline_ == nullptr) {
    return;
  }

  GstElementPtr appsink(gst_bin_get_by_name(GST_BIN(pipeline_.get()), kBridgeAppSinkName));
  if (appsink != nullptr && GST_IS_APP_SINK(appsink.get())) {
    GstAppSinkCallbacks callbacks{};
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink.get()), &callbacks, nullptr, nullptr);
  }

  GstBusPtr bus(gst_element_get_bus(pipeline_.get()));
  gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);
  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  if (result == GST_STATE_CHANGE_ASYNC) {
    (void)gst_element_get_state(pipeline_.get(), nullptr, nullptr, GST_CLOCK_TIME_NONE);
  }
  stopCallbacksAndWait();

  appsrc_.reset();
  pipeline_.reset();
}

GstFlowReturn GStreamerPipeline::onSample(GstAppSink * sink)
{
  if (!beginCallback()) {
    return GST_FLOW_FLUSHING;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  const auto callbacks = this->callbacks();
  GstSamplePtr sample(gst_app_sink_pull_sample(sink));
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample.get());
  const std::int64_t timestamp_us = (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer))
                                      ? static_cast<std::int64_t>(GST_BUFFER_PTS(buffer) / 1000U)
                                      : 0;

  livekit::VideoFrame frame;
  try {
    frame = unpackI420Frame(sample.get());
  } catch (const std::exception & exc) {
    callbacks.on_unpack_failed(exc.what());
    return GST_FLOW_ERROR;
  }

  if (callbacks.is_shutdown()) {
    return GST_FLOW_FLUSHING;
  }

  try {
    callbacks.on_frame(frame, timestamp_us);
    return GST_FLOW_OK;
  } catch (const std::exception & exc) {
    callbacks.on_capture_failed(exc.what());
    return GST_FLOW_ERROR;
  }
}

void GStreamerPipeline::onBusMessage(GstMessage * message)
{
  if (!beginCallback()) {
    return;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  const auto callbacks = this->callbacks();
  const GstMessageType type = GST_MESSAGE_TYPE(message);
  if (type == GST_MESSAGE_EOS) {
    callbacks.on_failed("eos");
    return;
  }
  if (type != GST_MESSAGE_ERROR) {
    return;
  }

  GError * raw_error = nullptr;
  gst_message_parse_error(message, &raw_error, nullptr);
  GErrorPtr error(raw_error);
  const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";
  callbacks.on_failed(reason);
}

}  // namespace livekit_ros2_bridge::video
