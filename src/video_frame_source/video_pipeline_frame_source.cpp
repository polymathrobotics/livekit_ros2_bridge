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

#include "video_frame_source/video_pipeline_frame_source.hpp"

#include <gst/video/video.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamRegistryLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");
constexpr char kAppSinkName[] = "bridge_video_sink";

struct PackedI420Frame
{
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> data;
};

void copyPlaneRows(const GstVideoFrame * frame, guint component_index, std::uint8_t * dst, std::size_t dst_stride)
{
  const auto * src = static_cast<const std::uint8_t *>(GST_VIDEO_FRAME_COMP_DATA(frame, component_index));
  const int src_stride = GST_VIDEO_FRAME_COMP_STRIDE(frame, component_index);
  const int plane_width = GST_VIDEO_FRAME_COMP_WIDTH(frame, component_index);
  const int plane_height = GST_VIDEO_FRAME_COMP_HEIGHT(frame, component_index);
  if (src == nullptr) {
    throw std::runtime_error("I420 frame plane data is unavailable.");
  }
  if (src_stride < 0 || plane_width <= 0 || plane_height <= 0) {
    throw std::runtime_error("I420 frame plane layout is invalid.");
  }
  if (static_cast<std::size_t>(plane_width) > dst_stride || src_stride < plane_width) {
    throw std::runtime_error("I420 frame plane stride is unsupported.");
  }

  const std::size_t src_stride_bytes = static_cast<std::size_t>(src_stride);
  const std::size_t plane_width_bytes = static_cast<std::size_t>(plane_width);
  for (int row = 0; row < plane_height; ++row) {
    std::memcpy(
      dst + static_cast<std::size_t>(row) * dst_stride,
      src + static_cast<std::size_t>(row) * src_stride_bytes,
      plane_width_bytes);
  }
}

PackedI420Frame copySampleToPackedI420(GstSample * sample)
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (caps == nullptr || buffer == nullptr) {
    throw std::runtime_error("GStreamer sample is missing caps or buffer.");
  }

  GstVideoInfo video_info;
  if (!gst_video_info_from_caps(&video_info, caps)) {
    throw std::runtime_error("Failed to parse GStreamer video caps.");
  }
  if (GST_VIDEO_INFO_FORMAT(&video_info) != GST_VIDEO_FORMAT_I420) {
    throw std::runtime_error("Video pipeline did not output I420 frames.");
  }

  PackedI420Frame frame;
  frame.width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&video_info));
  frame.height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&video_info));
  if (frame.width <= 0 || frame.height <= 0) {
    throw std::runtime_error("I420 sample dimensions are invalid.");
  }

  GstVideoFrameGuard mapped_frame(&video_info, buffer, GST_MAP_READ);
  if (!mapped_frame.is_valid()) {
    throw std::runtime_error("Failed to map GStreamer video frame.");
  }

  // The LiveKit C++ API accepts owned, tightly-packed frame bytes. Appsink may
  // hand us planar I420 with padding, so we repack planes row-by-row while
  // keeping the frame in I420 to avoid any CPU color conversion.
  const std::size_t luma_width = static_cast<std::size_t>(frame.width);
  const std::size_t luma_height = static_cast<std::size_t>(frame.height);
  const std::size_t chroma_width = (luma_width + 1U) / 2U;
  const std::size_t chroma_height = (luma_height + 1U) / 2U;
  const auto * gst_frame = mapped_frame.get();
  if (
    GST_VIDEO_FRAME_COMP_WIDTH(gst_frame, 0) != frame.width ||
    GST_VIDEO_FRAME_COMP_HEIGHT(gst_frame, 0) != frame.height ||
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_WIDTH(gst_frame, 1)) != chroma_width ||
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_HEIGHT(gst_frame, 1)) != chroma_height ||
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_WIDTH(gst_frame, 2)) != chroma_width ||
    static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_HEIGHT(gst_frame, 2)) != chroma_height)
  {
    throw std::runtime_error("Unexpected I420 plane dimensions from GStreamer.");
  }

  frame.data.resize(luma_width * luma_height + chroma_width * chroma_height * 2U);
  auto * y_plane = frame.data.data();
  auto * u_plane = y_plane + luma_width * luma_height;
  auto * v_plane = u_plane + chroma_width * chroma_height;
  copyPlaneRows(gst_frame, 0, y_plane, luma_width);
  copyPlaneRows(gst_frame, 1, u_plane, chroma_width);
  copyPlaneRows(gst_frame, 2, v_plane, chroma_width);
  return frame;
}

}  // namespace

std::string composeVideoPipeline(const std::string & ingress_fragment, const std::string & transform_fragment)
{
  std::string pipeline = ingress_fragment;
  if (!transform_fragment.empty()) {
    pipeline += " ! ";
    pipeline += transform_fragment;
  }
  pipeline += " ! queue max-size-buffers=2 leaky=downstream";
  pipeline += " ! videoconvert";
  // Emit encoder-native I420 so LiveKit/WebRTC does not have to do an extra
  // RGBA->I420 conversion on every captured frame.
  pipeline += " ! video/x-raw,format=I420";
  pipeline += " ! appsink name=";
  pipeline += kAppSinkName;
  pipeline += " sync=false drop=true max-buffers=1 emit-signals=false";
  return pipeline;
}

VideoPipelineFrameSource::VideoPipelineFrameSource(
  VideoStreamSpec spec, VideoFrameSink & frame_sink, VideoStreamLifecycleObserver & lifecycle_observer)
: spec_(std::move(spec))
, frame_sink_(frame_sink)
, lifecycle_observer_(lifecycle_observer)
{}

VideoPipelineFrameSource::~VideoPipelineFrameSource() = default;

GstFlowReturn VideoPipelineFrameSource::onNewSampleThunk(GstAppSink * sink, gpointer user_data)
{
  return static_cast<VideoPipelineFrameSource *>(user_data)->onNewSample(sink);
}

GstBusSyncReply VideoPipelineFrameSource::onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data)
{
  static_cast<VideoPipelineFrameSource *>(user_data)->onBusMessage(message);
  return GST_BUS_PASS;
}

VideoPipelineFrameSource::DetachedPipelineState VideoPipelineFrameSource::detachPipelineStateLocked()
{
  DetachedPipelineState detached;
  detached.pipeline = std::move(pipeline_);
  detached.appsrc = std::move(appsrc_);
  detached.appsink = std::move(appsink_);
  resetSourceStateLocked();
  first_sample_logged_ = false;
  return detached;
}

void VideoPipelineFrameSource::teardownDetachedPipelineState(DetachedPipelineState & detached)
{
  if (detached.appsink != nullptr) {
    GstAppSinkCallbacks callbacks{};
    gst_app_sink_set_callbacks(detached.appsink.get(), &callbacks, nullptr, nullptr);
  }
  if (detached.pipeline != nullptr) {
    GstBusPtr bus(gst_element_get_bus(detached.pipeline.get()));
    gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);
    gst_element_set_state(detached.pipeline.get(), GST_STATE_NULL);
  }

  detached.appsrc.reset();
  detached.appsink.reset();
  detached.pipeline.reset();
}

void VideoPipelineFrameSource::startPipelineLocked(const std::string & pipeline_description, bool expect_appsrc)
{
  ensureGstreamerInitialized();
  first_sample_logged_ = false;

  LogEvent(kVideoStreamRegistryLogger, "video_stream_pipeline_starting")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("ingest_mode", spec_.ingest_mode)
    .field("expect_appsrc", expect_appsrc)
    .field("pipeline", pipeline_description)
    .info();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(pipeline_description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  if (pipeline == nullptr) {
    const std::string message = error != nullptr ? error->message : "gst_parse_launch returned null";
    throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
  }

  if (!GST_IS_BIN(pipeline.get())) {
    throw std::runtime_error("Video pipeline must resolve to a GstBin.");
  }

  GstElementPtr sink(gst_bin_get_by_name(GST_BIN(pipeline.get()), kAppSinkName));
  if (sink == nullptr) {
    throw std::runtime_error("Video pipeline did not create the expected appsink.");
  }

  GstElementPtr src;
  if (expect_appsrc) {
    src.reset(gst_bin_get_by_name(GST_BIN(pipeline.get()), kVideoAppSrcName));
    if (src == nullptr) {
      throw std::runtime_error("Video pipeline did not create the expected appsrc.");
    }
  }

  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = &VideoPipelineFrameSource::onNewSampleThunk;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink.get()), &callbacks, this, nullptr);

  GstBusPtr bus(gst_element_get_bus(pipeline.get()));
  gst_bus_set_sync_handler(bus.get(), &VideoPipelineFrameSource::onBusMessageThunk, this, nullptr);

  pipeline_ = std::move(pipeline);
  appsink_.reset(GST_APP_SINK(sink.release()));
  appsrc_.reset(src == nullptr ? nullptr : GST_APP_SRC(src.release()));
}

void VideoPipelineFrameSource::playPipelineLocked()
{
  if (pipeline_ == nullptr) {
    throw std::runtime_error("Video pipeline is unavailable.");
  }

  const GstStateChangeReturn change = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (change != GST_STATE_CHANGE_FAILURE) {
    LogEvent(kVideoStreamRegistryLogger, "video_stream_pipeline_playing")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("state_change", static_cast<int>(change))
      .info();
    return;
  }

  discardPipelineElementsLocked();
  throw std::runtime_error("Failed to set video pipeline to PLAYING.");
}

void VideoPipelineFrameSource::discardPipelineElementsLocked()
{
  GstElementPtr pipeline = std::move(pipeline_);
  GstAppSrcPtr appsrc = std::move(appsrc_);
  GstAppSinkPtr appsink = std::move(appsink_);

  if (appsink != nullptr) {
    GstAppSinkCallbacks callbacks{};
    gst_app_sink_set_callbacks(appsink.get(), &callbacks, nullptr, nullptr);
  }
  if (pipeline != nullptr) {
    GstBusPtr bus(gst_element_get_bus(pipeline.get()));
    gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);
    gst_element_set_state(pipeline.get(), GST_STATE_NULL);
  }

  appsrc.reset();
  appsink.reset();
  pipeline.reset();
}

void VideoPipelineFrameSource::stopPipelineLocked()
{
  resetSourceStateLocked();
  first_sample_logged_ = false;
  discardPipelineElementsLocked();
}

bool VideoPipelineFrameSource::shouldRestartAfterFailure() const
{
  return false;
}

std::chrono::milliseconds VideoPipelineFrameSource::restartDelayOnFailure() const
{
  return std::chrono::milliseconds(0);
}

void VideoPipelineFrameSource::restartAfterFailureLocked()
{}

GstFlowReturn VideoPipelineFrameSource::onNewSample(GstAppSink * sink)
{
  GstSamplePtr sample(gst_app_sink_pull_sample(sink));
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  PackedI420Frame frame;
  try {
    frame = copySampleToPackedI420(sample.get());
  } catch (const std::exception & exc) {
    lifecycle_observer_.onVideoStreamSampleUnpackFailed(exc.what());
    return GST_FLOW_ERROR;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample.get());
  if (buffer == nullptr) {
    return GST_FLOW_ERROR;
  }
  const std::int64_t timestamp_us =
    GST_BUFFER_PTS_IS_VALID(buffer) ? static_cast<std::int64_t>(GST_BUFFER_PTS(buffer) / 1000U) : 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return GST_FLOW_FLUSHING;
    }

    if (!first_sample_logged_) {
      LogEvent(kVideoStreamRegistryLogger, "video_stream_sample_received")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("width", frame.width)
        .field("height", frame.height)
        .field("bytes", frame.data.size())
        .field("timestamp_us", timestamp_us)
        .info();
      first_sample_logged_ = true;
    }
  }

  try {
    frame_sink_.handleFrame(frame.width, frame.height, std::move(frame.data), timestamp_us);
    return GST_FLOW_OK;
  } catch (const std::exception & exc) {
    lifecycle_observer_.onVideoStreamCaptureFailed(exc.what());
    return GST_FLOW_ERROR;
  }
}

void VideoPipelineFrameSource::onBusMessage(GstMessage * message)
{
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_EOS:
      handlePipelineFailure("eos");
      break;
    case GST_MESSAGE_ERROR: {
      GError * raw_error = nullptr;
      gchar * raw_debug = nullptr;
      gst_message_parse_error(message, &raw_error, &raw_debug);
      GErrorPtr error(raw_error);
      GCharPtr debug(raw_debug);
      (void)debug;
      const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";
      handlePipelineFailure(reason);
      break;
    }
    default:
      break;
  }
}

void VideoPipelineFrameSource::handlePipelineFailure(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || pipeline_ == nullptr) {
    return;
  }

  lifecycle_observer_.onVideoStreamPipelineFailed(reason);

  if (failure_recovery_pending_) {
    return;
  }
  failure_recovery_pending_ = true;

  auto self = shared_from_this();
  const auto restart_delay = restartDelayOnFailure();
  std::thread([self, restart_delay]() {
    if (restart_delay.count() > 0) {
      std::this_thread::sleep_for(restart_delay);
    }
    self->recoverFromPipelineFailure();
  }).detach();
}

void VideoPipelineFrameSource::recoverFromPipelineFailure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  stopPipelineLocked();
  failure_recovery_pending_ = false;

  if (!shouldRestartAfterFailure()) {
    return;
  }

  try {
    restartAfterFailureLocked();
  } catch (const std::exception & exc) {
    lifecycle_observer_.onVideoStreamRestartFailed(exc.what());
  }
}

}  // namespace livekit_ros2_bridge
