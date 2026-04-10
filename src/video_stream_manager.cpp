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

#include "video_stream_manager.hpp"

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
#include "encoding_utils.hpp"
#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/log_event.hpp"

#include <gst/video/video-format.h>

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamManagerLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_manager");
constexpr char kAppSrcName[] = "bridge_video_src";
constexpr char kAppSinkName[] = "bridge_video_sink";
constexpr auto kExternalRestartDelay = std::chrono::milliseconds(250);

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

std::optional<std::string> parseCompressedFormat(const std::string & raw_format)
{
  const auto sep = raw_format.find(';');
  std::string primary = raw_format.substr(0, sep);
  primary.erase(
    primary.begin(), std::find_if(primary.begin(), primary.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  primary.erase(
    std::find_if(primary.rbegin(), primary.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
    primary.end());
  std::transform(primary.begin(), primary.end(), primary.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (primary == "jpeg" || primary == "jpg") {
    return std::string("jpeg");
  }
  if (primary == "png") {
    return std::string("png");
  }
  return std::nullopt;
}

GstClockTime rosStampToClockTime(const builtin_interfaces::msg::Time & stamp)
{
  const std::uint64_t sec = stamp.sec < 0 ? 0U : static_cast<std::uint64_t>(stamp.sec);
  return sec * GST_SECOND + stamp.nanosec;
}

RawSourceConfig makeRawSourceConfig(const sensor_msgs::msg::Image & message)
{
  const GstVideoFormat format = rosEncodingToGstFormat(message.encoding);
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

std::string composePipeline(const std::string & prefix, const std::string & middle)
{
  std::string pipeline = prefix;
  if (!middle.empty()) {
    pipeline += " ! ";
    pipeline += middle;
  }
  pipeline += " ! queue max-size-buffers=2 leaky=downstream";
  pipeline += " ! videoconvert";
  pipeline += " ! video/x-raw,format=RGBA";
  pipeline += " ! appsink name=";
  pipeline += kAppSinkName;
  pipeline += " sync=false drop=true max-buffers=1 emit-signals=false";
  return pipeline;
}

}  // namespace

class VideoStreamManager::StreamRecord final : public std::enable_shared_from_this<StreamRecord>
{
public:
  StreamRecord(rclcpp::Node & node, RoomSession & session, SidecarLaunchSpec spec)
  : node_(node)
  , session_(session)
  , spec_(std::move(spec))
  {}

  ~StreamRecord()
  {
    shutdown();
  }

  std::string ensureRunning()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (spec_.source_kind == VideoSourceKind::Pipeline) {
      if (pipeline_ == nullptr) {
        startExternalPipelineLocked();
      }
    } else if (!subscription_) {
      createRosSubscriptionLocked();
    }

    return spec_.track_name;
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    subscription_.reset();
    stopPipelineLocked();
  }

private:
  static GstFlowReturn onNewSampleThunk(GstAppSink * sink, gpointer user_data)
  {
    return static_cast<StreamRecord *>(user_data)->onNewSample(sink);
  }

  static GstBusSyncReply onBusMessageThunk(GstBus *, GstMessage * message, gpointer user_data)
  {
    static_cast<StreamRecord *>(user_data)->onBusMessage(message);
    return GST_BUS_PASS;
  }

  void createRosSubscriptionLocked()
  {
    if (spec_.interface_type == kImageInterfaceType) {
      auto weak_self = weak_from_this();
      subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
        spec_.ros_topic, rclcpp::SensorDataQoS(), [weak_self](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          if (const auto self = weak_self.lock()) {
            self->handleRawImageMessage(message);
          }
        });
      LogEvent(kVideoStreamManagerLogger, "video_stream_subscription_started")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("topic", spec_.ros_topic)
        .kv("interface_type", spec_.interface_type)
        .info();
      return;
    }

    if (spec_.interface_type == kCompressedImageInterfaceType) {
      auto weak_self = weak_from_this();
      subscription_ = node_.create_subscription<sensor_msgs::msg::CompressedImage>(
        spec_.ros_topic,
        rclcpp::SensorDataQoS(),
        [weak_self](const sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
          if (const auto self = weak_self.lock()) {
            self->handleCompressedImageMessage(message);
          }
        });
      LogEvent(kVideoStreamManagerLogger, "video_stream_subscription_started")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("topic", spec_.ros_topic)
        .kv("interface_type", spec_.interface_type)
        .info();
      return;
    }

    throw std::runtime_error("Unsupported ROS video interface type '" + spec_.interface_type + "'.");
  }

  void handleRawImageMessage(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    try {
      if (!first_input_logged_) {
        LogEvent(kVideoStreamManagerLogger, "video_stream_input_received")
          .kv("stream_key", spec_.stream_key)
          .kv("track_name", spec_.track_name)
          .kv("encoding", message->encoding)
          .kv("width", message->width)
          .kv("height", message->height)
          .kv("step", message->step)
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
      LogEvent(kVideoStreamManagerLogger, "video_stream_push_failed")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("error", exc.what())
        .warn();
      stopPipelineLocked();
    }
  }

  void handleCompressedImageMessage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    try {
      if (!first_input_logged_) {
        LogEvent(kVideoStreamManagerLogger, "video_stream_input_received")
          .kv("stream_key", spec_.stream_key)
          .kv("track_name", spec_.track_name)
          .kv("format", message->format)
          .kv("bytes", message->data.size())
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
      LogEvent(kVideoStreamManagerLogger, "video_stream_push_failed")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("error", exc.what())
        .warn();
      stopPipelineLocked();
    }
  }

  void startExternalPipelineLocked()
  {
    const std::string prefix = spec_.pipeline_description;
    startPipelineLocked(composePipeline(prefix, ""));
    playPipelineLocked();
  }

  void startRawRosPipelineLocked(const RawSourceConfig & config)
  {
    std::string prefix = "appsrc name=";
    prefix += kAppSrcName;
    prefix += " is-live=true block=false format=time do-timestamp=true";
    prefix += " caps=";
    prefix += formatToCapsString(config);
    startPipelineLocked(composePipeline(prefix, spec_.pipeline_description), true);
    playPipelineLocked();
    raw_source_config_ = config;
  }

  void startCompressedRosPipelineLocked(const std::string & format)
  {
    std::string prefix = "appsrc name=";
    prefix += kAppSrcName;
    prefix += " is-live=true block=false format=time do-timestamp=true";
    prefix += format == "png" ? " caps=image/png ! pngdec" : " caps=image/jpeg ! jpegdec";
    startPipelineLocked(composePipeline(prefix, spec_.pipeline_description), true);
    playPipelineLocked();
    compressed_format_ = format;
  }

  void startPipelineLocked(const std::string & pipeline_description, bool expect_appsrc = false)
  {
    ensureGstreamerInitialized();
    first_sample_logged_ = false;

    LogEvent(kVideoStreamManagerLogger, "video_stream_pipeline_starting")
      .kv("stream_key", spec_.stream_key)
      .kv("track_name", spec_.track_name)
      .kv("ingest_mode", spec_.ingest_mode)
      .kv("expect_appsrc", expect_appsrc)
      .kv("pipeline", pipeline_description)
      .info();

    GError * error = nullptr;
    GstElement * pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
    if (pipeline == nullptr) {
      const std::string message = error != nullptr ? error->message : "gst_parse_launch returned null";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
    }

    if (!GST_IS_BIN(pipeline)) {
      gst_object_unref(pipeline);
      throw std::runtime_error("Video pipeline must resolve to a GstBin.");
    }

    GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline), kAppSinkName);
    if (sink == nullptr) {
      gst_object_unref(pipeline);
      throw std::runtime_error("Video pipeline did not create the expected appsink.");
    }

    GstElement * src = nullptr;
    if (expect_appsrc) {
      src = gst_bin_get_by_name(GST_BIN(pipeline), kAppSrcName);
      if (src == nullptr) {
        gst_object_unref(sink);
        gst_object_unref(pipeline);
        throw std::runtime_error("Video pipeline did not create the expected appsrc.");
      }
    }

    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = &StreamRecord::onNewSampleThunk;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);

    GstBus * bus = gst_element_get_bus(pipeline);
    gst_bus_set_sync_handler(bus, &StreamRecord::onBusMessageThunk, this, nullptr);
    gst_object_unref(bus);

    pipeline_ = pipeline;
    appsink_ = GST_APP_SINK(sink);
    appsrc_ = src == nullptr ? nullptr : GST_APP_SRC(src);
  }

  void playPipelineLocked()
  {
    if (pipeline_ == nullptr) {
      throw std::runtime_error("Video pipeline is unavailable.");
    }

    const GstStateChangeReturn change = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (change != GST_STATE_CHANGE_FAILURE) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_pipeline_playing")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("state_change", static_cast<int>(change))
        .info();
      return;
    }

    discardPipelineElementsLocked();
    throw std::runtime_error("Failed to set video pipeline to PLAYING.");
  }

  void discardPipelineElementsLocked()
  {
    GstElement * pipeline = pipeline_;
    GstAppSrc * appsrc = appsrc_;
    GstAppSink * appsink = appsink_;
    pipeline_ = nullptr;
    appsrc_ = nullptr;
    appsink_ = nullptr;

    if (appsink != nullptr) {
      GstAppSinkCallbacks callbacks{};
      gst_app_sink_set_callbacks(appsink, &callbacks, nullptr, nullptr);
    }
    if (pipeline != nullptr) {
      GstBus * bus = gst_element_get_bus(pipeline);
      gst_bus_set_sync_handler(bus, nullptr, nullptr, nullptr);
      gst_object_unref(bus);
      gst_element_set_state(pipeline, GST_STATE_NULL);
    }
    if (appsrc != nullptr) {
      gst_object_unref(appsrc);
    }
    if (appsink != nullptr) {
      gst_object_unref(appsink);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
  }

  void stopPipelineLocked()
  {
    if (published_track_) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_track_unpublishing")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .info();
      session_.unpublishVideoTrack(published_track_);
      published_track_.reset();
    }
    video_source_.reset();
    published_width_ = 0;
    published_height_ = 0;
    raw_source_config_.reset();
    compressed_format_.clear();
    first_sample_logged_ = false;

    discardPipelineElementsLocked();
  }

  void pushRawImageLocked(const sensor_msgs::msg::Image & message, const RawSourceConfig & config)
  {
    if (appsrc_ == nullptr) {
      throw std::runtime_error("Raw video appsrc is unavailable.");
    }

    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, message.data.size(), nullptr);
    if (buffer == nullptr) {
      throw std::runtime_error("Failed to allocate GStreamer buffer.");
    }

    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      gst_buffer_unref(buffer);
      throw std::runtime_error("Failed to map GStreamer buffer.");
    }
    std::memcpy(map.data, message.data.data(), message.data.size());
    gst_buffer_unmap(buffer, &map);

    gsize offsets[GST_VIDEO_MAX_PLANES] = {0};
    gint strides[GST_VIDEO_MAX_PLANES] = {static_cast<gint>(config.stride)};
    (void)gst_buffer_add_video_meta_full(
      buffer,
      GST_VIDEO_FRAME_FLAG_NONE,
      config.format,
      static_cast<guint>(config.width),
      static_cast<guint>(config.height),
      1,
      offsets,
      strides);

    const auto pts = rosStampToClockTime(message.header.stamp);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc_, buffer);
    if (result != GST_FLOW_OK) {
      throw std::runtime_error("Failed to push raw ROS image into GStreamer.");
    }
  }

  void pushCompressedImageLocked(const sensor_msgs::msg::CompressedImage & message)
  {
    if (appsrc_ == nullptr) {
      throw std::runtime_error("Compressed video appsrc is unavailable.");
    }

    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, message.data.size(), nullptr);
    if (buffer == nullptr) {
      throw std::runtime_error("Failed to allocate GStreamer buffer.");
    }

    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      gst_buffer_unref(buffer);
      throw std::runtime_error("Failed to map GStreamer buffer.");
    }
    std::memcpy(map.data, message.data.data(), message.data.size());
    gst_buffer_unmap(buffer, &map);

    const auto pts = rosStampToClockTime(message.header.stamp);
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc_, buffer);
    if (result != GST_FLOW_OK) {
      throw std::runtime_error("Failed to push compressed ROS image into GStreamer.");
    }
  }

  GstFlowReturn onNewSample(GstAppSink * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
      return GST_FLOW_EOS;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (caps == nullptr || buffer == nullptr) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstStructure * structure = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (!gst_structure_get_int(structure, "width", &width) || !gst_structure_get_int(structure, "height", &height)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    std::vector<std::uint8_t> rgba(map.data, map.data + map.size);
    const std::int64_t timestamp_us =
      GST_BUFFER_PTS_IS_VALID(buffer) ? static_cast<std::int64_t>(GST_BUFFER_PTS(buffer) / 1000U) : 0;
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return GST_FLOW_FLUSHING;
      }

      if (!first_sample_logged_) {
        LogEvent(kVideoStreamManagerLogger, "video_stream_sample_received")
          .kv("stream_key", spec_.stream_key)
          .kv("track_name", spec_.track_name)
          .kv("width", width)
          .kv("height", height)
          .kv("bytes", rgba.size())
          .kv("timestamp_us", timestamp_us)
          .info();
        first_sample_logged_ = true;
      }

      ensurePublishedTrackLocked(width, height);
      livekit::VideoFrame frame(width, height, livekit::VideoBufferType::RGBA, std::move(rgba));
      video_source_->captureFrame(frame, timestamp_us);
      return GST_FLOW_OK;
    } catch (const std::exception & exc) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_capture_failed")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("error", exc.what())
        .warn();
      return GST_FLOW_ERROR;
    }
  }

  void ensurePublishedTrackLocked(int width, int height)
  {
    if (
      video_source_ != nullptr && published_track_ != nullptr && published_width_ == width &&
      published_height_ == height)
    {
      return;
    }

    const bool republishing = published_track_ != nullptr;
    if (published_track_) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_track_replacing")
        .kv("stream_key", spec_.stream_key)
        .kv("track_name", spec_.track_name)
        .kv("previous_width", published_width_)
        .kv("previous_height", published_height_)
        .kv("next_width", width)
        .kv("next_height", height)
        .info();
      session_.unpublishVideoTrack(published_track_);
      published_track_.reset();
    }

    video_source_ = std::make_shared<livekit::VideoSource>(width, height);
    published_track_ = session_.publishVideoTrack(spec_.track_name, video_source_);
    published_width_ = width;
    published_height_ = height;

    LogEvent(
      kVideoStreamManagerLogger, republishing ? "video_stream_track_republished" : "video_stream_track_published")
      .kv("stream_key", spec_.stream_key)
      .kv("track_name", spec_.track_name)
      .kv("width", width)
      .kv("height", height)
      .info();
  }

  void onBusMessage(GstMessage * message)
  {
    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_EOS:
        handlePipelineFailure("eos");
        break;
      case GST_MESSAGE_ERROR: {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";
        if (error != nullptr) {
          g_error_free(error);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
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

    LogEvent(kVideoStreamManagerLogger, "video_stream_pipeline_failed")
      .kv("stream_key", spec_.stream_key)
      .kv("track_name", spec_.track_name)
      .kv("reason", reason)
      .warn();

    if (failure_recovery_pending_) {
      return;
    }
    failure_recovery_pending_ = true;

    const bool restart_external = spec_.source_kind == VideoSourceKind::Pipeline;
    auto self = shared_from_this();
    std::thread([self, restart_external]() {
      if (restart_external) {
        std::this_thread::sleep_for(kExternalRestartDelay);
      }
      self->recoverFromPipelineFailure(restart_external);
    }).detach();
  }

  void recoverFromPipelineFailure(bool restart_external)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    stopPipelineLocked();
    failure_recovery_pending_ = false;

    if (restart_external) {
      try {
        startExternalPipelineLocked();
      } catch (const std::exception & exc) {
        LogEvent(kVideoStreamManagerLogger, "video_stream_restart_failed")
          .kv("stream_key", spec_.stream_key)
          .kv("track_name", spec_.track_name)
          .kv("error", exc.what())
          .warn();
      }
    }
  }

  rclcpp::Node & node_;
  RoomSession & session_;
  SidecarLaunchSpec spec_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  bool failure_recovery_pending_ = false;
  bool first_input_logged_ = false;
  bool first_sample_logged_ = false;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  GstElement * pipeline_ = nullptr;
  GstAppSrc * appsrc_ = nullptr;
  GstAppSink * appsink_ = nullptr;
  std::optional<RawSourceConfig> raw_source_config_;
  std::string compressed_format_;
  std::shared_ptr<livekit::VideoSource> video_source_;
  std::shared_ptr<PublishedVideoTrack> published_track_;
  int published_width_ = 0;
  int published_height_ = 0;
};

VideoStreamManager::VideoStreamManager(rclcpp::Node & node, RoomSession & session)
: node_(node)
, session_(session)
{
  ensureGstreamerInitialized();
}

VideoStreamManager::~VideoStreamManager()
{
  shutdown();
}

std::string VideoStreamManager::ensureStream(const SidecarLaunchSpec & spec)
{
  std::shared_ptr<StreamRecord> record;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream manager is shut down.");
    }

    auto [it, inserted] = streams_.try_emplace(spec.stream_key);
    if (inserted) {
      it->second = std::make_shared<StreamRecord>(node_, session_, spec);
    }
    record = it->second;
  }

  return record->ensureRunning();
}

void VideoStreamManager::stopStream(const std::string & stream_key)
{
  std::shared_ptr<StreamRecord> record;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = streams_.find(stream_key);
    if (it == streams_.end()) {
      return;
    }
    record = std::move(it->second);
    streams_.erase(it);
  }

  if (record) {
    record->shutdown();
  }
}

void VideoStreamManager::shutdown()
{
  std::unordered_map<std::string, std::shared_ptr<StreamRecord>> streams;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    is_shutdown_ = true;
    streams = std::move(streams_);
    streams_.clear();
  }

  for (auto & entry : streams) {
    if (entry.second) {
      entry.second->shutdown();
    }
  }
}

}  // namespace livekit_ros2_bridge
