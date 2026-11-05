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

#include "video/gstreamer_pipeline.hpp"

#include <gst/video/video.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "livekit/video_source.h"
#include "utils/scope_exit.hpp"
#include "video/h264_access_unit.hpp"
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

livekit::EncodedVideoSource::Frame unpackH264Frame(GstSample * sample, std::int64_t timestamp_us)
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (caps == nullptr || buffer == nullptr || gst_caps_get_size(caps) == 0U) {
    throw std::runtime_error("GStreamer sample is missing caps or buffer.");
  }

  const GstStructure * structure = gst_caps_get_structure(caps, 0U);
  int width = 0;
  int height = 0;
  if (
    !gst_structure_get_int(structure, "width", &width) || !gst_structure_get_int(structure, "height", &height) ||
    width <= 0 || height <= 0)
  {
    throw std::runtime_error("H.264 sample caps are missing valid dimensions.");
  }

  GstBufferMap mapping(*buffer, GST_MAP_READ);
  if (!mapping.is_valid()) {
    throw std::runtime_error("Failed to map GStreamer H.264 buffer.");
  }

  const auto * data = static_cast<const std::uint8_t *>(mapping.get()->data);
  const std::size_t size = mapping.get()->size;
  livekit::EncodedVideoSource::Frame frame;
  frame.is_keyframe = containsIdrSlice(data, size);
  frame.width = static_cast<std::uint32_t>(width);
  frame.height = static_cast<std::uint32_t>(height);
  frame.timestamp_us = timestamp_us;
  frame.data.assign(data, data + size);
  return frame;
}

GstFlowReturn deliverEncodedSample(GstSample * sample, std::int64_t timestamp_us, const PipelineCallbacks & callbacks)
{
  livekit::EncodedVideoSource::Frame frame;
  try {
    frame = unpackH264Frame(sample, timestamp_us);
  } catch (const std::exception & exc) {
    callbacks.on_unpack_failed(exc.what());
    return GST_FLOW_ERROR;
  }

  if (callbacks.is_shutdown()) {
    return GST_FLOW_FLUSHING;
  }

  try {
    callbacks.on_encoded_frame(std::move(frame));
    return GST_FLOW_OK;
  } catch (const std::exception & exc) {
    callbacks.on_capture_failed(exc.what());
    return GST_FLOW_ERROR;
  }
}

// Not-linked and not-negotiated both mean an element's output did not fit the
// next element; in passthrough that is usually a non-H.264 source meeting h264parse.
bool isCapsMismatch(GstMessage * message, const GError * error)
{
  if (
    error != nullptr && (g_error_matches(error, GST_CORE_ERROR, GST_CORE_ERROR_NEGOTIATION) ||
                         g_error_matches(error, GST_STREAM_ERROR, GST_STREAM_ERROR_FORMAT) ||
                         g_error_matches(error, GST_STREAM_ERROR, GST_STREAM_ERROR_WRONG_TYPE)))
  {
    return true;
  }

  // GST_ELEMENT_FLOW_ERROR records the failing flow return in the error details.
  const GstStructure * details = nullptr;
  gst_message_parse_error_details(message, &details);
  int flow_return = GST_FLOW_OK;
  if (details == nullptr || !gst_structure_get_int(details, "flow-return", &flow_return)) {
    return false;
  }
  return flow_return == GST_FLOW_NOT_LINKED || flow_return == GST_FLOW_NOT_NEGOTIATED;
}

// Best effort: caps left on unlinked source pads of top-level elements are what
// the source offered when it could not link onward. Joined with "; ".
std::string findUnlinkedSourceCaps(GstBin * bin)
{
  std::string offered_caps;
  GstIteratorPtr iterator(gst_bin_iterate_elements(bin));
  (void)gst_iterator_foreach(
    iterator.get(),
    [](const GValue * item, gpointer user_data) {
      auto * element = GST_ELEMENT(g_value_get_object(item));
      (void)gst_element_foreach_src_pad(
        element,
        [](GstElement *, GstPad * pad, gpointer pad_user_data) -> gboolean {
          if (gst_pad_is_linked(pad)) {
            return TRUE;
          }
          utils::GstCapsPtr caps(gst_pad_get_current_caps(pad));
          if (caps == nullptr) {
            return TRUE;
          }
          GCharPtr caps_text(gst_caps_to_string(caps.get()));
          if (caps_text == nullptr) {
            return TRUE;
          }
          auto & found_caps = *static_cast<std::string *>(pad_user_data);
          found_caps += found_caps.empty() ? "" : "; ";
          found_caps += caps_text.get();
          return TRUE;
        },
        user_data);
    },
    &offered_caps);
  return offered_caps;
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

void GStreamerPipeline::start(const std::string & description, bool require_appsrc, PipelineOutput output)
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

  // Encoded frames can't be dropped, so the bridge queue blocks when full; its
  // overrun is the signal that the consumer fell behind.
  gulong overrun_handler = 0U;
  if (output == PipelineOutput::EncodedH264) {
    GstElementPtr queue(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeVideoQueueName));
    if (queue == nullptr) {
      throw std::runtime_error("Video pipeline did not create the expected queue.");
    }
    overrun_handler = g_signal_connect(
      queue.get(),
      "overrun",
      G_CALLBACK(
        +[](GstElement *, gpointer user_data) { static_cast<GStreamerPipeline *>(user_data)->onQueueOverrun(); }),
      this);
    if (overrun_handler == 0U) {
      throw std::runtime_error(std::string("Video pipeline named ") + kBridgeVideoQueueName + " must be a queue.");
    }
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
  output_ = output;
  overrun_handler_ = overrun_handler;
  key_frame_pad_.store(gst_element_get_static_pad(GST_ELEMENT(appsink.get()), "sink"), std::memory_order_release);

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

  // Released at return, after in-flight requestKeyFrame() calls finish.
  const GstObjectPtr<GstPad> key_frame_pad(key_frame_pad_.exchange(nullptr, std::memory_order_acq_rel));

  GstElementPtr appsink(gst_bin_get_by_name(GST_BIN(pipeline_.get()), kBridgeAppSinkName));
  if (appsink != nullptr && GST_IS_APP_SINK(appsink.get())) {
    GstAppSinkCallbacks callbacks{};
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink.get()), &callbacks, nullptr, nullptr);
  }

  if (overrun_handler_ != 0U) {
    GstElementPtr queue(gst_bin_get_by_name(GST_BIN(pipeline_.get()), kBridgeVideoQueueName));
    if (queue != nullptr) {
      g_signal_handler_disconnect(queue.get(), overrun_handler_);
    }
    overrun_handler_ = 0U;
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

void GStreamerPipeline::requestKeyFrame()
{
  if (!beginCallback()) {
    return;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  GstPad * pad = key_frame_pad_.load(std::memory_order_acquire);
  if (pad == nullptr) {
    return;
  }

  // Push from the sink pad rather than gst_element_send_event(), which takes the
  // appsink's state lock that a concurrent stop() holds while it waits for this thread.
  (void)gst_pad_push_event(pad, gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0U));
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

  if (output_ == PipelineOutput::EncodedH264) {
    return deliverEncodedSample(sample.get(), timestamp_us, callbacks);
  }

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

void GStreamerPipeline::onQueueOverrun()
{
  if (!beginCallback()) {
    return;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  const auto callbacks = this->callbacks();
  callbacks.on_fell_behind();
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
  if (output_ == PipelineOutput::EncodedH264 && callbacks.on_caps_mismatch && isCapsMismatch(message, error.get())) {
    callbacks.on_caps_mismatch(reason, findUnlinkedSourceCaps(GST_BIN(pipeline_.get())));
    return;
  }
  callbacks.on_failed(reason);
}

}  // namespace livekit_ros2_bridge::video
