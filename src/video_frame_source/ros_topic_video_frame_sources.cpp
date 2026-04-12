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

#include "video_frame_source/ros_topic_video_frame_sources.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "utils/log_event.hpp"
#include "utils/trim.hpp"
#include "video_frame_source.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamRegistryLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

GstVideoFormat rosImageEncodingToGstFormat(const std::string & encoding)
{
  // Keep this intentionally limited to known raw formats instead of
  // guessing caps for other ROS encodings.
  if (encoding == sensor_msgs::image_encodings::MONO8) return GST_VIDEO_FORMAT_GRAY8;
  if (encoding == sensor_msgs::image_encodings::MONO16) return GST_VIDEO_FORMAT_GRAY16_LE;
  if (encoding == sensor_msgs::image_encodings::RGB8) return GST_VIDEO_FORMAT_RGB;
  if (encoding == sensor_msgs::image_encodings::BGR8) return GST_VIDEO_FORMAT_BGR;
  if (encoding == sensor_msgs::image_encodings::RGBA8) return GST_VIDEO_FORMAT_RGBA;
  if (encoding == sensor_msgs::image_encodings::BGRA8) return GST_VIDEO_FORMAT_BGRA;
  if (encoding == sensor_msgs::image_encodings::YUV422) return GST_VIDEO_FORMAT_UYVY;
  if (encoding == sensor_msgs::image_encodings::YUV422_YUY2) return GST_VIDEO_FORMAT_YUY2;
  return GST_VIDEO_FORMAT_UNKNOWN;
}

std::optional<std::string> normalizeCompressedCodecToken(std::string token)
{
  token = trim(token);
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

}  // namespace

bool operator==(const RawSourceConfig & lhs, const RawSourceConfig & rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.format == rhs.format && lhs.stride == rhs.stride;
}

bool operator!=(const RawSourceConfig & lhs, const RawSourceConfig & rhs)
{
  return !(lhs == rhs);
}

RawRosVideoFrameSource::RawRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer)
: VideoPipelineFrameSource(std::move(spec), frame_sink, lifecycle_observer)
, node_(node)
, subscription_qos_config_(subscription_qos_config)
{}

void RawRosVideoFrameSource::ensureRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (!subscription_) {
    createRosSubscriptionLocked();
  }
}

void RawRosVideoFrameSource::shutdown()
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

void RawRosVideoFrameSource::createRosSubscriptionLocked()
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
    std::weak_ptr<RawRosVideoFrameSource>(std::static_pointer_cast<RawRosVideoFrameSource>(shared_from_this()));
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

void RawRosVideoFrameSource::handleRawImageMessage(const sensor_msgs::msg::Image::ConstSharedPtr & message)
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
    lifecycle_observer_.onVideoStreamPushFailed(exc.what());
    stopPipelineLocked();
  }
}

void RawRosVideoFrameSource::startRawRosPipelineLocked(const RawSourceConfig & config)
{
  std::string prefix = "appsrc name=";
  prefix += kVideoAppSrcName;
  prefix += " is-live=true block=false format=time do-timestamp=true";
  prefix += " caps=";
  prefix += formatToCapsString(config);
  startPipelineLocked(composeVideoPipeline(prefix, spec_.transform_fragment), true);
  playPipelineLocked();
  raw_source_config_ = config;
}

void RawRosVideoFrameSource::pushRawImageLocked(const sensor_msgs::msg::Image & message, const RawSourceConfig & config)
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

void RawRosVideoFrameSource::resetSourceStateLocked()
{
  raw_source_config_.reset();
}

CompressedRosVideoFrameSource::CompressedRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer)
: VideoPipelineFrameSource(std::move(spec), frame_sink, lifecycle_observer)
, node_(node)
, subscription_qos_config_(subscription_qos_config)
{}

void CompressedRosVideoFrameSource::ensureRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (!subscription_) {
    createRosSubscriptionLocked();
  }
}

void CompressedRosVideoFrameSource::shutdown()
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

void CompressedRosVideoFrameSource::createRosSubscriptionLocked()
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

  auto weak_self = std::weak_ptr<CompressedRosVideoFrameSource>(
    std::static_pointer_cast<CompressedRosVideoFrameSource>(shared_from_this()));
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

void CompressedRosVideoFrameSource::handleCompressedImageMessage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message)
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
    lifecycle_observer_.onVideoStreamPushFailed(exc.what());
    stopPipelineLocked();
  }
}

void CompressedRosVideoFrameSource::startCompressedRosPipelineLocked(const std::string & format)
{
  std::string prefix = "appsrc name=";
  prefix += kVideoAppSrcName;
  prefix += " is-live=true block=false format=time do-timestamp=true";
  prefix += format == "png" ? " caps=image/png ! pngdec" : " caps=image/jpeg ! jpegdec";
  startPipelineLocked(composeVideoPipeline(prefix, spec_.transform_fragment), true);
  playPipelineLocked();
  compressed_format_ = format;
}

void CompressedRosVideoFrameSource::pushCompressedImageLocked(const sensor_msgs::msg::CompressedImage & message)
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

void CompressedRosVideoFrameSource::resetSourceStateLocked()
{
  compressed_format_.clear();
}

std::shared_ptr<VideoFrameSource> makeRawRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer)
{
  return std::make_shared<RawRosVideoFrameSource>(
    node, std::move(spec), subscription_qos_config, frame_sink, lifecycle_observer);
}

std::shared_ptr<VideoFrameSource> makeCompressedRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer)
{
  return std::make_shared<CompressedRosVideoFrameSource>(
    node, std::move(spec), subscription_qos_config, frame_sink, lifecycle_observer);
}

}  // namespace livekit_ros2_bridge
