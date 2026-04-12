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

#include "video_ingestor.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "ros_image_format_mapping.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_qos.hpp"
#include "utils/gstreamer_raii.hpp"
#include "utils/log_event.hpp"

#include <gst/video/video-format.h>

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamRegistryLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");
constexpr char kAppSrcName[] = "bridge_video_src";
constexpr char kAppSinkName[] = "bridge_video_sink";
constexpr auto kConfiguredSourceRestartDelay = std::chrono::milliseconds(250);

using GstAppSrcPtr = GstObjectPtr<GstAppSrc>;
using GstAppSinkPtr = GstObjectPtr<GstAppSink>;

struct PackedI420Frame
{
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> data;
};

void ensureGstreamerInitialized()
{
  static std::once_flag once;
  std::call_once(once, []() { gst_init(nullptr, nullptr); });
}

struct RawSourceConfig
{
  int width = 0;
  int height = 0;
  GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
  std::uint32_t stride = 0;
};

bool operator==(const RawSourceConfig & lhs, const RawSourceConfig & rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.format == rhs.format && lhs.stride == rhs.stride;
}

bool operator!=(const RawSourceConfig & lhs, const RawSourceConfig & rhs)
{
  return !(lhs == rhs);
}

std::string trimWhitespace(std::string value)
{
  value.erase(
    value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  value.erase(
    std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), value.end());
  return value;
}

std::optional<std::string> normalizeCompressedCodecToken(std::string token)
{
  token = trimWhitespace(std::move(token));
  const auto token_end = token.find_first_of(" \t\r\n");
  if (token_end != std::string::npos) {
    token.resize(token_end);
  }
  std::transform(
    token.begin(), token.end(), token.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (token == "jpeg" || token == "jpg") {
    return std::string("jpeg");
  }
  if (token == "png") {
    return std::string("png");
  }
  return std::nullopt;
}

std::optional<std::string> parseCompressedFormat(const std::string & raw_format)
{
  const auto sep = raw_format.find(';');
  if (const auto primary = normalizeCompressedCodecToken(raw_format.substr(0, sep)); primary.has_value()) {
    return primary;
  }
  if (sep == std::string::npos) {
    return std::nullopt;
  }
  return normalizeCompressedCodecToken(raw_format.substr(sep + 1));
}

GstClockTime rosStampToClockTime(const builtin_interfaces::msg::Time & stamp)
{
  const std::uint64_t sec = stamp.sec < 0 ? 0U : static_cast<std::uint64_t>(stamp.sec);
  return sec * GST_SECOND + stamp.nanosec;
}

RawSourceConfig makeRawSourceConfig(const sensor_msgs::msg::Image & message)
{
  const GstVideoFormat format = rosImageEncodingToGstFormat(message.encoding);
  if (format == GST_VIDEO_FORMAT_UNKNOWN) {
    throw std::runtime_error("Unsupported ROS image encoding '" + message.encoding + "'.");
  }

  RawSourceConfig config;
  config.width = static_cast<int>(message.width);
  config.height = static_cast<int>(message.height);
  config.format = format;
  config.stride = message.step;
  return config;
}

std::string formatToCapsString(const RawSourceConfig & config)
{
  const char * format_name = gst_video_format_to_string(config.format);
  if (format_name == nullptr) {
    throw std::runtime_error("Unsupported GStreamer video format.");
  }

  return std::string("video/x-raw,format=") + format_name + ",width=" + std::to_string(config.width) +
         ",height=" + std::to_string(config.height) + ",framerate=0/1";
}

std::string composeBridgeOwnedVideoPipeline(
  const std::string & ingress_fragment, const std::string & transform_fragment)
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

class BridgeOwnedVideoPipelineIngestor : public IVideoIngestor,
                                         public std::enable_shared_from_this<BridgeOwnedVideoPipelineIngestor>
{
public:
  BridgeOwnedVideoPipelineIngestor(VideoStreamSpec spec, IVideoFrameSink & frame_sink)
  : spec_(std::move(spec))
  , frame_sink_(frame_sink)
  {}

  virtual ~BridgeOwnedVideoPipelineIngestor() = default;

protected:
  struct DetachedPipelineState
  {
    GstElementPtr pipeline;
    GstAppSrcPtr appsrc;
    GstAppSinkPtr appsink;
  };

  static GstFlowReturn onNewSampleThunk(GstAppSink * sink, gpointer user_data)
  {
    return static_cast<BridgeOwnedVideoPipelineIngestor *>(user_data)->onNewSample(sink);
  }

  static GstBusSyncReply onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data)
  {
    static_cast<BridgeOwnedVideoPipelineIngestor *>(user_data)->onBusMessage(message);
    return GST_BUS_PASS;
  }

  DetachedPipelineState detachPipelineStateLocked()
  {
    DetachedPipelineState detached;
    detached.pipeline = std::move(pipeline_);
    detached.appsrc = std::move(appsrc_);
    detached.appsink = std::move(appsink_);
    resetSourceStateLocked();
    first_sample_logged_ = false;
    return detached;
  }

  static void teardownDetachedPipelineState(DetachedPipelineState & detached)
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

  void startPipelineLocked(const std::string & pipeline_description, bool expect_appsrc = false)
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
      src.reset(gst_bin_get_by_name(GST_BIN(pipeline.get()), kAppSrcName));
      if (src == nullptr) {
        throw std::runtime_error("Video pipeline did not create the expected appsrc.");
      }
    }

    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = &BridgeOwnedVideoPipelineIngestor::onNewSampleThunk;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink.get()), &callbacks, this, nullptr);

    GstBusPtr bus(gst_element_get_bus(pipeline.get()));
    gst_bus_set_sync_handler(bus.get(), &BridgeOwnedVideoPipelineIngestor::onBusMessageThunk, this, nullptr);

    pipeline_ = std::move(pipeline);
    appsink_.reset(GST_APP_SINK(sink.release()));
    appsrc_.reset(src == nullptr ? nullptr : GST_APP_SRC(src.release()));
  }

  void playPipelineLocked()
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

  void discardPipelineElementsLocked()
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

  void stopPipelineLocked()
  {
    resetSourceStateLocked();
    first_sample_logged_ = false;
    discardPipelineElementsLocked();
  }

  virtual void resetSourceStateLocked() = 0;

  virtual bool shouldRestartAfterFailure() const
  {
    return false;
  }

  virtual std::chrono::milliseconds restartDelayOnFailure() const
  {
    return std::chrono::milliseconds(0);
  }

  virtual void restartAfterFailureLocked()
  {}

  VideoStreamSpec spec_;
  IVideoFrameSink & frame_sink_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  bool failure_recovery_pending_ = false;
  bool first_sample_logged_ = false;
  GstElementPtr pipeline_;
  GstAppSrcPtr appsrc_;
  GstAppSinkPtr appsink_;

private:
  GstFlowReturn onNewSample(GstAppSink * sink)
  {
    GstSamplePtr sample(gst_app_sink_pull_sample(sink));
    if (sample == nullptr) {
      return GST_FLOW_EOS;
    }

    PackedI420Frame frame;
    try {
      frame = copySampleToPackedI420(sample.get());
    } catch (const std::exception & exc) {
      LogEvent(kVideoStreamRegistryLogger, "video_stream_sample_unpack_failed")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("error", exc.what())
        .warn();
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
      LogEvent(kVideoStreamRegistryLogger, "video_stream_capture_failed")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("error", exc.what())
        .warn();
      return GST_FLOW_ERROR;
    }
  }

  void onBusMessage(GstMessage * message)
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

  void handlePipelineFailure(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_ || pipeline_ == nullptr) {
      return;
    }

    LogEvent(kVideoStreamRegistryLogger, "video_stream_pipeline_failed")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("reason", reason)
      .warn();

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

  void recoverFromPipelineFailure()
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
      LogEvent(kVideoStreamRegistryLogger, "video_stream_restart_failed")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("error", exc.what())
        .warn();
    }
  }
};

class RawRosVideoIngestor final : public BridgeOwnedVideoPipelineIngestor
{
public:
  RawRosVideoIngestor(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config,
    IVideoFrameSink & frame_sink)
  : BridgeOwnedVideoPipelineIngestor(std::move(spec), frame_sink)
  , node_(node)
  , subscription_qos_config_(subscription_qos_config)
  {}

  void ensureRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (!subscription_) {
      createRosSubscriptionLocked();
    }
  }

  void shutdown() override
  {
    DetachedPipelineState detached_pipeline_state;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return;
      }

      is_shutdown_ = true;
      subscription = std::move(subscription_);
      detached_pipeline_state = detachPipelineStateLocked();
    }

    subscription.reset();
    teardownDetachedPipelineState(detached_pipeline_state);
  }

private:
  void createRosSubscriptionLocked()
  {
    const rclcpp::QoS base_qos(rclcpp::KeepLast(1));
    const ResolvedSubscriptionQos resolved_qos =
      resolveTopicSubscriptionQos(node_, spec_.ros_topic, base_qos, subscription_qos_config_);

    LogEvent(kVideoStreamRegistryLogger, "subscription_qos_resolved")
      .field("resource", spec_.ros_topic)
      .field("kind", "topic")
      .field("delivery", "video")
      .field("interface_type", spec_.interface_type)
      .field("source", subscriptionQosResolutionSourceToString(resolved_qos.source))
      .field("reliability", reliabilityPolicyToString(resolved_qos.qos.reliability()))
      .field("durability", durabilityPolicyToString(resolved_qos.qos.durability()))
      .field("used_publisher_qos", resolved_qos.used_publisher_qos)
      .field("mixed_reliability", resolved_qos.mixed_reliability)
      .field("mixed_durability", resolved_qos.mixed_durability)
      .field("override_id", resolved_qos.matched_override_id)
      .field("override_pattern", resolved_qos.matched_override_pattern)
      .info();

    auto weak_self =
      std::weak_ptr<RawRosVideoIngestor>(std::static_pointer_cast<RawRosVideoIngestor>(shared_from_this()));
    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      spec_.ros_topic, resolved_qos.qos, [weak_self](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        if (const auto self = weak_self.lock()) {
          self->handleRawImageMessage(message);
        }
      });
    LogEvent(kVideoStreamRegistryLogger, "video_stream_subscription_started")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("topic", spec_.ros_topic)
      .field("interface_type", spec_.interface_type)
      .info();
  }

  void handleRawImageMessage(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    try {
      if (!first_input_logged_) {
        LogEvent(kVideoStreamRegistryLogger, "video_stream_input_received")
          .field("stream_key", spec_.stream_key)
          .field("track_name", spec_.track_name)
          .field("encoding", message->encoding)
          .field("width", message->width)
          .field("height", message->height)
          .field("step", message->step)
          .info();
        first_input_logged_ = true;
      }

      const RawSourceConfig config = makeRawSourceConfig(*message);
      if (!raw_source_config_.has_value() || *raw_source_config_ != config || pipeline_ == nullptr) {
        stopPipelineLocked();
        startRawRosPipelineLocked(config);
      }

      pushRawImageLocked(*message, config);
    } catch (const std::exception & exc) {
      LogEvent(kVideoStreamRegistryLogger, "video_stream_push_failed")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("error", exc.what())
        .warn();
      stopPipelineLocked();
    }
  }

  void startRawRosPipelineLocked(const RawSourceConfig & config)
  {
    std::string prefix = "appsrc name=";
    prefix += kAppSrcName;
    prefix += " is-live=true block=false format=time do-timestamp=true";
    prefix += " caps=";
    prefix += formatToCapsString(config);
    startPipelineLocked(composeBridgeOwnedVideoPipeline(prefix, spec_.transform_fragment), true);
    playPipelineLocked();
    raw_source_config_ = config;
  }

  void pushRawImageLocked(const sensor_msgs::msg::Image & message, const RawSourceConfig & config)
  {
    if (appsrc_ == nullptr) {
      throw std::runtime_error("Raw video appsrc is unavailable.");
    }

    GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, message.data.size(), nullptr));
    if (buffer == nullptr) {
      throw std::runtime_error("Failed to allocate GStreamer buffer.");
    }

    {
      GstMapGuard map(buffer.get(), GST_MAP_WRITE);
      if (!map.is_valid()) {
        throw std::runtime_error("Failed to map GStreamer buffer.");
      }
      std::memcpy(map.get()->data, message.data.data(), message.data.size());
    }

    gsize offsets[GST_VIDEO_MAX_PLANES] = {0};
    gint strides[GST_VIDEO_MAX_PLANES] = {static_cast<gint>(config.stride)};
    (void)gst_buffer_add_video_meta_full(
      buffer.get(),
      GST_VIDEO_FRAME_FLAG_NONE,
      config.format,
      static_cast<guint>(config.width),
      static_cast<guint>(config.height),
      1,
      offsets,
      strides);

    const auto pts = rosStampToClockTime(message.header.stamp);
    GST_BUFFER_PTS(buffer.get()) = pts;
    GST_BUFFER_DTS(buffer.get()) = pts;
    GST_BUFFER_DURATION(buffer.get()) = GST_CLOCK_TIME_NONE;

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc_.get(), buffer.release());
    if (result != GST_FLOW_OK) {
      throw std::runtime_error("Failed to push raw ROS image into GStreamer.");
    }
  }

  void resetSourceStateLocked() override
  {
    raw_source_config_.reset();
  }

  rclcpp::Node & node_;
  const SubscriptionQosConfig * subscription_qos_config_;
  bool first_input_logged_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::optional<RawSourceConfig> raw_source_config_;
};

class CompressedRosVideoIngestor final : public BridgeOwnedVideoPipelineIngestor
{
public:
  CompressedRosVideoIngestor(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config,
    IVideoFrameSink & frame_sink)
  : BridgeOwnedVideoPipelineIngestor(std::move(spec), frame_sink)
  , node_(node)
  , subscription_qos_config_(subscription_qos_config)
  {}

  void ensureRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (!subscription_) {
      createRosSubscriptionLocked();
    }
  }

  void shutdown() override
  {
    DetachedPipelineState detached_pipeline_state;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return;
      }

      is_shutdown_ = true;
      subscription = std::move(subscription_);
      detached_pipeline_state = detachPipelineStateLocked();
    }

    subscription.reset();
    teardownDetachedPipelineState(detached_pipeline_state);
  }

private:
  void createRosSubscriptionLocked()
  {
    const rclcpp::QoS base_qos(rclcpp::KeepLast(1));
    const ResolvedSubscriptionQos resolved_qos =
      resolveTopicSubscriptionQos(node_, spec_.ros_topic, base_qos, subscription_qos_config_);

    LogEvent(kVideoStreamRegistryLogger, "subscription_qos_resolved")
      .field("resource", spec_.ros_topic)
      .field("kind", "topic")
      .field("delivery", "video")
      .field("interface_type", spec_.interface_type)
      .field("source", subscriptionQosResolutionSourceToString(resolved_qos.source))
      .field("reliability", reliabilityPolicyToString(resolved_qos.qos.reliability()))
      .field("durability", durabilityPolicyToString(resolved_qos.qos.durability()))
      .field("used_publisher_qos", resolved_qos.used_publisher_qos)
      .field("mixed_reliability", resolved_qos.mixed_reliability)
      .field("mixed_durability", resolved_qos.mixed_durability)
      .field("override_id", resolved_qos.matched_override_id)
      .field("override_pattern", resolved_qos.matched_override_pattern)
      .info();

    auto weak_self = std::weak_ptr<CompressedRosVideoIngestor>(
      std::static_pointer_cast<CompressedRosVideoIngestor>(shared_from_this()));
    subscription_ = node_.create_subscription<sensor_msgs::msg::CompressedImage>(
      spec_.ros_topic, resolved_qos.qos, [weak_self](const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        if (const auto self = weak_self.lock()) {
          self->handleCompressedImageMessage(message);
        }
      });
    LogEvent(kVideoStreamRegistryLogger, "video_stream_subscription_started")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("topic", spec_.ros_topic)
      .field("interface_type", spec_.interface_type)
      .info();
  }

  void handleCompressedImageMessage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    try {
      if (!first_input_logged_) {
        LogEvent(kVideoStreamRegistryLogger, "video_stream_input_received")
          .field("stream_key", spec_.stream_key)
          .field("track_name", spec_.track_name)
          .field("format", message->format)
          .field("bytes", message->data.size())
          .info();
        first_input_logged_ = true;
      }

      const auto format = parseCompressedFormat(message->format);
      if (!format.has_value()) {
        throw std::runtime_error("Unsupported compressed image format '" + message->format + "'.");
      }

      if (compressed_format_ != *format || pipeline_ == nullptr) {
        stopPipelineLocked();
        startCompressedRosPipelineLocked(*format);
      }

      pushCompressedImageLocked(*message);
    } catch (const std::exception & exc) {
      LogEvent(kVideoStreamRegistryLogger, "video_stream_push_failed")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("error", exc.what())
        .warn();
      stopPipelineLocked();
    }
  }

  void startCompressedRosPipelineLocked(const std::string & format)
  {
    std::string prefix = "appsrc name=";
    prefix += kAppSrcName;
    prefix += " is-live=true block=false format=time do-timestamp=true";
    prefix += format == "png" ? " caps=image/png ! pngdec" : " caps=image/jpeg ! jpegdec";
    startPipelineLocked(composeBridgeOwnedVideoPipeline(prefix, spec_.transform_fragment), true);
    playPipelineLocked();
    compressed_format_ = format;
  }

  void pushCompressedImageLocked(const sensor_msgs::msg::CompressedImage & message)
  {
    if (appsrc_ == nullptr) {
      throw std::runtime_error("Compressed video appsrc is unavailable.");
    }

    GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, message.data.size(), nullptr));
    if (buffer == nullptr) {
      throw std::runtime_error("Failed to allocate GStreamer buffer.");
    }

    {
      GstMapGuard map(buffer.get(), GST_MAP_WRITE);
      if (!map.is_valid()) {
        throw std::runtime_error("Failed to map GStreamer buffer.");
      }
      std::memcpy(map.get()->data, message.data.data(), message.data.size());
    }

    const auto pts = rosStampToClockTime(message.header.stamp);
    GST_BUFFER_PTS(buffer.get()) = pts;
    GST_BUFFER_DTS(buffer.get()) = pts;
    GST_BUFFER_DURATION(buffer.get()) = GST_CLOCK_TIME_NONE;

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc_.get(), buffer.release());
    if (result != GST_FLOW_OK) {
      throw std::runtime_error("Failed to push compressed ROS image into GStreamer.");
    }
  }

  void resetSourceStateLocked() override
  {
    compressed_format_.clear();
  }

  rclcpp::Node & node_;
  const SubscriptionQosConfig * subscription_qos_config_;
  bool first_input_logged_ = false;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  std::string compressed_format_;
};

class ConfiguredSourceVideoIngestor final : public BridgeOwnedVideoPipelineIngestor
{
public:
  ConfiguredSourceVideoIngestor(VideoStreamSpec spec, IVideoFrameSink & frame_sink)
  : BridgeOwnedVideoPipelineIngestor(std::move(spec), frame_sink)
  {}

  void ensureRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (pipeline_ == nullptr) {
      startConfiguredSourceIngestLocked();
    }
  }

  void shutdown() override
  {
    DetachedPipelineState detached_pipeline_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return;
      }

      is_shutdown_ = true;
      detached_pipeline_state = detachPipelineStateLocked();
    }

    teardownDetachedPipelineState(detached_pipeline_state);
  }

private:
  void startConfiguredSourceIngestLocked()
  {
    startPipelineLocked(composeBridgeOwnedVideoPipeline(spec_.ingress_fragment, spec_.transform_fragment));
    playPipelineLocked();
  }

  void resetSourceStateLocked() override
  {}

  bool shouldRestartAfterFailure() const override
  {
    return true;
  }

  std::chrono::milliseconds restartDelayOnFailure() const override
  {
    return kConfiguredSourceRestartDelay;
  }

  void restartAfterFailureLocked() override
  {
    startConfiguredSourceIngestLocked();
  }
};

}  // namespace

std::shared_ptr<IVideoIngestor> makeRawRosVideoIngestor(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  IVideoFrameSink & frame_sink)
{
  return std::make_shared<RawRosVideoIngestor>(node, std::move(spec), subscription_qos_config, frame_sink);
}

std::shared_ptr<IVideoIngestor> makeCompressedRosVideoIngestor(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  IVideoFrameSink & frame_sink)
{
  return std::make_shared<CompressedRosVideoIngestor>(node, std::move(spec), subscription_qos_config, frame_sink);
}

std::shared_ptr<IVideoIngestor> makeConfiguredSourceVideoIngestor(VideoStreamSpec spec, IVideoFrameSink & frame_sink)
{
  return std::make_shared<ConfiguredSourceVideoIngestor>(std::move(spec), frame_sink);
}

}  // namespace livekit_ros2_bridge
