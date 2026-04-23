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

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.video_pipeline_frame_source");
constexpr auto kOtherVideoRestartDelay = std::chrono::milliseconds(250);

struct PackedI420Frame
{
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> data;
};

struct I420Layout
{
  int width = 0;
  int height = 0;

  static I420Layout fromInfo(const GstVideoInfo & video_info)
  {
    I420Layout layout;
    layout.width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&video_info));
    layout.height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&video_info));
    if (layout.width <= 0 || layout.height <= 0) {
      throw std::runtime_error("I420 sample dimensions are invalid.");
    }
    return layout;
  }

  [[nodiscard]] std::size_t lumaWidth() const
  {
    return static_cast<std::size_t>(width);
  }

  [[nodiscard]] std::size_t lumaHeight() const
  {
    return static_cast<std::size_t>(height);
  }

  [[nodiscard]] std::size_t chromaWidth() const
  {
    return (lumaWidth() + 1U) / 2U;
  }

  [[nodiscard]] std::size_t chromaHeight() const
  {
    return (lumaHeight() + 1U) / 2U;
  }

  [[nodiscard]] std::size_t lumaPlaneSize() const
  {
    return lumaWidth() * lumaHeight();
  }

  [[nodiscard]] std::size_t chromaPlaneSize() const
  {
    return chromaWidth() * chromaHeight();
  }

  [[nodiscard]] std::size_t byteCount() const
  {
    return lumaPlaneSize() + chromaPlaneSize() * 2U;
  }

  void validate(const GstVideoFrame * frame) const
  {
    if (
      !hasPlaneDimensions(frame, 0, lumaWidth(), lumaHeight()) ||
      !hasPlaneDimensions(frame, 1, chromaWidth(), chromaHeight()) ||
      !hasPlaneDimensions(frame, 2, chromaWidth(), chromaHeight()))
    {
      throw std::runtime_error("Unexpected I420 plane dimensions from GStreamer.");
    }
  }

private:
  static bool hasPlaneDimensions(
    const GstVideoFrame * frame, guint component_index, std::size_t expected_width, std::size_t expected_height)
  {
    return static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_WIDTH(frame, component_index)) == expected_width &&
           static_cast<std::size_t>(GST_VIDEO_FRAME_COMP_HEIGHT(frame, component_index)) == expected_height;
  }
};

void copyI420Plane(const GstVideoFrame * frame, guint component_index, std::uint8_t * dst, std::size_t dst_stride)
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

PackedI420Frame packI420Frame(GstSample * sample)
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

  const I420Layout layout = I420Layout::fromInfo(video_info);

  GstVideoFrameGuard mapped_frame(&video_info, buffer, GST_MAP_READ);
  if (!mapped_frame.is_valid()) {
    throw std::runtime_error("Failed to map GStreamer video frame.");
  }

  // The LiveKit C++ API accepts owned, tightly-packed frame bytes. Appsink may
  // hand us planar I420 with padding, so we repack planes row-by-row while
  // keeping the frame in I420 to avoid any CPU color conversion.
  const auto * gst_frame = mapped_frame.get();
  layout.validate(gst_frame);

  PackedI420Frame frame;
  frame.width = layout.width;
  frame.height = layout.height;
  frame.data.resize(layout.byteCount());

  auto * dst = frame.data.data();
  copyI420Plane(gst_frame, 0, dst, layout.lumaWidth());
  copyI420Plane(gst_frame, 1, dst + layout.lumaPlaneSize(), layout.chromaWidth());
  copyI420Plane(gst_frame, 2, dst + layout.lumaPlaneSize() + layout.chromaPlaneSize(), layout.chromaWidth());
  return frame;
}

}  // namespace

VideoPipelineFrameSource::VideoPipelineFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  std::optional<RestartConfig> restart_config)
: spec_(std::move(spec))
, sink_(sink)
, observer_(observer)
, restart_config_(std::move(restart_config))
{}

VideoPipelineFrameSource::~VideoPipelineFrameSource()
{
  VideoPipelineFrameSource::close();
}

void VideoPipelineFrameSource::activateFixedPipeline()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (pipeline_ != nullptr) {
    return;
  }

  if (!restart_config_.has_value()) {
    throw std::logic_error("Video pipeline source activation requires a fixed restart config.");
  }

  const auto & config = restart_config_.value();
  startPipelineLocked(fixedDescription(), config.require_appsrc);
}

void VideoPipelineFrameSource::close()
{
  PipelineHandles handles;
  bool recovery_pending = false;
  std::thread recovery_thread;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    recovery_pending = recovery_pending_;
    recovery_thread = beginShutdownLocked();
  }

  joinRecoveryThread(recovery_thread);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Drop the internal handles while holding mutex_ so any in-flight callbacks
    // observe the terminal shutdown state before GStreamer teardown removes
    // callbacks and transitions the pipeline to NULL.
    handles = takePipelineLocked();
  }

  if (!recovery_pending && handles.pipeline == nullptr && handles.appsrc == nullptr && handles.appsink == nullptr) {
    return;
  }

  if (handles.pipeline != nullptr || recovery_pending) {
    LogEvent(kLogger, "video_stream_source_shutdown")
      .field("stream_key", spec_.stream_key)
      .fieldIf(recovery_pending, "restart_pending", true)
      .info();
  }

  teardown(handles.pipeline, handles.appsrc, handles.appsink);
}

VideoPipelineFrameSource::PipelineHandles VideoPipelineFrameSource::takePipelineLocked()
{
  resetLocked();
  recovery_pending_ = false;

  PipelineHandles handles;
  handles.pipeline = std::move(pipeline_);
  handles.appsrc = std::move(appsrc_);
  handles.appsink = std::move(appsink_);
  return handles;
}

std::thread VideoPipelineFrameSource::beginShutdownLocked()
{
  is_shutdown_ = true;
  recovery_condition_.notify_all();
  return std::move(recovery_thread_);
}

void VideoPipelineFrameSource::joinRecoveryThread(std::thread & recovery_thread)
{
  if (!recovery_thread.joinable()) {
    return;
  }
  recovery_thread.join();
}

void VideoPipelineFrameSource::startPipelineLocked(const std::string & description, bool require_appsrc)
{
  ensureGstreamerInitialized();

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

  PipelineHandles handles;
  handles.pipeline = std::move(pipeline);
  GstElementPtr appsink(gst_bin_get_by_name(GST_BIN(handles.pipeline.get()), kBridgeAppSinkName));
  if (appsink == nullptr) {
    throw std::runtime_error("Video pipeline did not create the expected appsink.");
  }
  if (!GST_IS_APP_SINK(appsink.get())) {
    throw std::runtime_error(std::string("Video pipeline named ") + kBridgeAppSinkName + " must be a GstAppSink.");
  }
  handles.appsink = GstAppSinkPtr(GST_APP_SINK(appsink.release()));

  if (require_appsrc) {
    GstElementPtr appsrc(gst_bin_get_by_name(GST_BIN(handles.pipeline.get()), kBridgeAppSrcName));
    if (appsrc == nullptr) {
      throw std::runtime_error("Video pipeline did not create the expected appsrc.");
    }
    if (!GST_IS_APP_SRC(appsrc.get())) {
      throw std::runtime_error(std::string("Video pipeline named ") + kBridgeAppSrcName + " must be a GstAppSrc.");
    }
    handles.appsrc = GstAppSrcPtr(GST_APP_SRC(appsrc.release()));
  }

  // Register callbacks before moving to PLAYING so startup-time samples or bus
  // errors are still routed through this instance.
  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = &VideoPipelineFrameSource::onSampleThunk;
  gst_app_sink_set_callbacks(handles.appsink.get(), &callbacks, this, nullptr);

  GstBusPtr bus(gst_element_get_bus(handles.pipeline.get()));
  gst_bus_set_sync_handler(bus.get(), &VideoPipelineFrameSource::onBusMessageThunk, this, nullptr);

  pipeline_ = std::move(handles.pipeline);
  appsink_ = std::move(handles.appsink);
  appsrc_ = std::move(handles.appsrc);

  const GstStateChangeReturn state_change = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (state_change == GST_STATE_CHANGE_FAILURE) {
    teardown(pipeline_, appsrc_, appsink_);
    throw std::runtime_error("Failed to set video pipeline to PLAYING.");
  }

  LogEvent(kLogger, "video_stream_pipeline_playing")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .info();
}

std::string VideoPipelineFrameSource::fixedDescription() const
{
  const auto & input = requireOtherVideoInput(spec_);
  return buildPipelineDescription(input.ingress_fragment, input.transform_fragment);
}

void VideoPipelineFrameSource::resetLocked()
{}

void VideoPipelineFrameSource::teardown(GstElementPtr & pipeline, GstAppSrcPtr & appsrc, GstAppSinkPtr & appsink)
{
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

GstFlowReturn VideoPipelineFrameSource::onSampleThunk(GstAppSink * sink, gpointer user_data)
{
  return static_cast<VideoPipelineFrameSource *>(user_data)->onSample(sink);
}

GstFlowReturn VideoPipelineFrameSource::onSample(GstAppSink * sink)
{
  GstSamplePtr sample(gst_app_sink_pull_sample(sink));
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample.get());
  const std::int64_t timestamp_us = (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer))
                                      ? static_cast<std::int64_t>(GST_BUFFER_PTS(buffer) / 1000U)
                                      : 0;

  PackedI420Frame frame;
  try {
    frame = packI420Frame(sample.get());
  } catch (const std::exception & exc) {
    observer_.onSampleUnpackFailed(exc.what());
    return GST_FLOW_ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return GST_FLOW_FLUSHING;
    }
  }

  try {
    // The mutex only protects local lifecycle state. Writing to the sink may
    // block in downstream LiveKit code, so keep that handoff outside the lock.
    sink_.write(frame.width, frame.height, std::move(frame.data), timestamp_us);
    return GST_FLOW_OK;
  } catch (const std::exception & exc) {
    observer_.onCaptureFailed(exc.what());
    return GST_FLOW_ERROR;
  }
}

GstBusSyncReply VideoPipelineFrameSource::onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data)
{
  static_cast<VideoPipelineFrameSource *>(user_data)->onBusMessage(message);
  return GST_BUS_PASS;
}

void VideoPipelineFrameSource::onBusMessage(GstMessage * message)
{
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_EOS:
      handleFailure("eos");
      return;
    case GST_MESSAGE_ERROR:
      break;
    default:
      return;
  }

  GError * raw_error = nullptr;
  gchar * raw_debug = nullptr;
  gst_message_parse_error(message, &raw_error, &raw_debug);
  GErrorPtr error(raw_error);
  GCharPtr debug(raw_debug);
  (void)debug;
  const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";
  handleFailure(reason);
}

void VideoPipelineFrameSource::handleFailure(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || pipeline_ == nullptr) {
    return;
  }

  if (recovery_pending_) {
    return;
  }
  recovery_pending_ = true;

  const RestartConfig * config = restart_config_ ? &*restart_config_ : nullptr;
  const auto restart_delay = config != nullptr ? config->restart_delay : std::chrono::milliseconds::zero();
  LogEvent(
    kLogger, config != nullptr ? "video_stream_pipeline_recovery_scheduled" : "video_stream_pipeline_recovery_disabled")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("reason", reason)
    .fieldIf(restart_delay > std::chrono::milliseconds::zero(), "restart_delay_ms", restart_delay.count())
    .warn();

  ensureRecoveryThreadLocked();
  recovery_condition_.notify_one();
}

void VideoPipelineFrameSource::ensureRecoveryThreadLocked()
{
  if (recovery_thread_.joinable()) {
    return;
  }

  recovery_thread_ = std::thread([this]() { recoveryLoop(); });
}

void VideoPipelineFrameSource::recoveryLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    recovery_condition_.wait(lock, [this]() { return is_shutdown_ || recovery_pending_; });
    if (is_shutdown_) {
      return;
    }

    const RestartConfig * config = restart_config_ ? &*restart_config_ : nullptr;
    const auto restart_delay = config != nullptr ? config->restart_delay : std::chrono::milliseconds::zero();
    if (restart_delay > std::chrono::milliseconds::zero()) {
      const bool cancelled = recovery_condition_.wait_for(lock, restart_delay, [this]() { return is_shutdown_; });
      if (cancelled) {
        return;
      }
    }

    lock.unlock();
    recoverAfterFailure();
    lock.lock();
  }
}

void VideoPipelineFrameSource::recoverAfterFailure()
{
  // Serialize teardown and replacement startup with the base mutex so
  // close() or producer-side reconfiguration never observes half-installed
  // pipeline members during recovery.
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  auto handles = takePipelineLocked();
  teardown(handles.pipeline, handles.appsrc, handles.appsink);
  // takePipelineLocked() clears the pending marker for the failed pipeline,
  // so a restarted pipeline begins from a clean recovery state.

  const RestartConfig * config = restart_config_ ? &*restart_config_ : nullptr;
  if (config == nullptr) {
    return;
  }

  try {
    startPipelineLocked(fixedDescription(), config->require_appsrc);
  } catch (const std::exception & exc) {
    observer_.onRestartFailed(exc.what());
  }
}

std::shared_ptr<VideoFrameSource> makeOtherVideoFrameSource(
  VideoStreamSpec spec, VideoFrameSink & sink, VideoStreamLifecycleObserver & observer)
{
  auto source = std::make_shared<VideoPipelineFrameSource>(
    std::move(spec),
    sink,
    observer,
    VideoPipelineFrameSource::RestartConfig{
      false,
      kOtherVideoRestartDelay,
    });
  source->activateFixedPipeline();
  return source;
}

}  // namespace livekit_ros2_bridge
