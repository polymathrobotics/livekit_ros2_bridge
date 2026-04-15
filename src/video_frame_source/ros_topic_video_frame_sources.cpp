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

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

const char * gstVideoFormatName(GstVideoFormat format)
{
  const char * format_name = gst_video_format_to_string(format);
  return format_name != nullptr ? format_name : "unknown";
}

GstVideoFormat gstFormatFromRosEncoding(const std::string & encoding)
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

const char * compressedImageCodecName(CompressedImageCodec codec)
{
  switch (codec) {
    case CompressedImageCodec::kJpeg:
      return "jpeg";
    case CompressedImageCodec::kPng:
      return "png";
    default:
      return "unknown";
  }
}

std::optional<CompressedImageCodec> parseCompressedImageCodec(const std::string & format)
{
  // `sensor_msgs/msg/CompressedImage::format` is convention-based rather than
  // strongly typed, so normalize the leading codec token and ignore any extra
  // transport-specific suffixes such as `jpeg; quality=...`.
  const auto parse_token = [](std::string token) -> std::optional<CompressedImageCodec> {
    token = trim(token);
    const auto token_end = token.find_first_of(" \t\r\n");
    if (token_end != std::string::npos) {
      token.resize(token_end);
    }
    std::transform(
      token.begin(), token.end(), token.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (token == "jpeg" || token == "jpg") return CompressedImageCodec::kJpeg;
    if (token == "png") return CompressedImageCodec::kPng;
    return std::nullopt;
  };

  const auto sep = format.find(';');
  if (const auto primary = parse_token(format.substr(0, sep)); primary.has_value()) {
    return primary;
  }
  if (sep == std::string::npos) {
    return std::nullopt;
  }
  return parse_token(format.substr(sep + 1));
}

std::optional<std::int64_t> timestampUsFromRosStamp(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || (stamp.sec == 0 && stamp.nanosec == 0U)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000LL + static_cast<std::int64_t>(stamp.nanosec / 1000U);
}

FrameLayout frameLayoutFromImage(const sensor_msgs::msg::Image & image, GstVideoFormat format)
{
  FrameLayout layout;
  layout.width = static_cast<int>(image.width);
  layout.height = static_cast<int>(image.height);
  layout.format = format;
  layout.stride = image.step;
  return layout;
}

bool frameLayoutsMatch(const FrameLayout & lhs, const FrameLayout & rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.format == rhs.format && lhs.stride == rhs.stride;
}

void logFrameLayoutChange(const VideoStreamSpec & spec, const FrameLayout & previous_layout, const FrameLayout & layout)
{
  LogEvent event(kLogger, "video_stream_input_layout_changed");
  event.field("stream_key", spec.stream_key).field("topic", spec.ros_topic);
  if (previous_layout.width != layout.width) {
    event.field("previous_width", previous_layout.width).field("width", layout.width);
  }
  if (previous_layout.height != layout.height) {
    event.field("previous_height", previous_layout.height).field("height", layout.height);
  }
  if (previous_layout.stride != layout.stride) {
    event.field("previous_stride", previous_layout.stride).field("stride", layout.stride);
  }
  if (previous_layout.format != layout.format) {
    event.field("previous_format", gstVideoFormatName(previous_layout.format))
      .field("format", gstVideoFormatName(layout.format));
  }
  event.info();
}

CompressedImageCodec compressedImageCodecFromImage(const sensor_msgs::msg::CompressedImage & image)
{
  const auto codec = parseCompressedImageCodec(image.format);
  if (!codec.has_value()) {
    throw std::runtime_error("Unsupported compressed image format '" + image.format + "'.");
  }
  return *codec;
}

GstBufferPtr makeStampedGstBuffer(
  const std::uint8_t * data, std::size_t size, const builtin_interfaces::msg::Time & stamp)
{
  // appsrc consumes buffers asynchronously, so copy the ROS payload into an owned GstBuffer.
  GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, size, nullptr));
  if (buffer == nullptr) {
    throw std::runtime_error("Failed to allocate GStreamer buffer.");
  }

  {
    GstMapGuard map(buffer.get(), GST_MAP_WRITE);
    if (!map.is_valid()) {
      throw std::runtime_error("Failed to map GStreamer buffer.");
    }
    if (size > 0U) {
      std::memcpy(map.get()->data, data, size);
    }
  }

  // GStreamer still expects a non-negative running-time timestamp on every
  // buffer, while profiling uses `nullopt` to mean "source time unavailable".
  const std::uint64_t sec = stamp.sec < 0 ? 0U : static_cast<std::uint64_t>(stamp.sec);
  const GstClockTime pts = sec * GST_SECOND + stamp.nanosec;
  GST_BUFFER_PTS(buffer.get()) = pts;
  GST_BUFFER_DTS(buffer.get()) = pts;
  GST_BUFFER_DURATION(buffer.get()) = GST_CLOCK_TIME_NONE;
  return buffer;
}

void logSubscriptionQos(const VideoStreamSpec & spec, const ResolvedSubscriptionQos & qos)
{
  LogEvent(kLogger, "subscription_qos_resolved")
    .field("resource", spec.ros_topic)
    .field("delivery", "video")
    .field("interface_type", spec.interface_type)
    .field("publisher_count", qos.publisher_count)
    .field("source", subscriptionQosSourceString(qos.source))
    .field("reliability", subscriptionQosReliabilityString(qos.qos.reliability()))
    .field("durability", subscriptionQosDurabilityString(qos.qos.durability()))
    .fieldIf(qos.used_publisher_qos, "used_publisher_qos", true)
    .fieldIf(qos.mixed_reliability, "mixed_reliability", true)
    .fieldIf(qos.mixed_durability, "mixed_durability", true)
    .fieldIfNotEmpty("override_id", qos.override_id)
    .fieldIfNotEmpty("override_pattern", qos.override_pattern)
    .info();
}

}  // namespace

RawRosVideoFrameSource::RawRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: VideoPipelineFrameSource(std::move(spec), sink, observer, std::move(profiler))
, node_(node)
, qos_config_(qos_config)
{}

void RawRosVideoFrameSource::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (subscription_) {
    return;
  }

  const rclcpp::QoS base_qos(rclcpp::KeepLast(1));
  ResolvedSubscriptionQos qos = resolveSubscriptionQos(node_, spec_.ros_topic, base_qos, qos_config_);
  logSubscriptionQos(spec_, qos);

  // The ROS subscription may still have queued callbacks during shutdown; the
  // weak ref prevents that queue from extending this source's lifetime.
  auto weak_self =
    std::weak_ptr<RawRosVideoFrameSource>(std::static_pointer_cast<RawRosVideoFrameSource>(shared_from_this()));
  subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
    spec_.ros_topic, qos.qos, [weak_self](const sensor_msgs::msg::Image::ConstSharedPtr image) {
      if (const auto self = weak_self.lock(); self) {
        self->onImage(image);
      }
    });
}

void RawRosVideoFrameSource::shutdown()
{
  PipelineHandles handles;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    subscription = std::move(subscription_);
    handles = takePipelineLocked();
  }

  // Release ROS/GStreamer objects after unlocking so any destruction-triggered
  // callbacks never re-enter this source while mutex_ is held.
  subscription.reset();
  teardown(handles.pipeline, handles.appsrc, handles.appsink);
}

void RawRosVideoFrameSource::onImage(const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  const auto ingress_time = VideoStreamProfiler::SteadyClock::now();
  const auto source_timestamp_us = timestampUsFromRosStamp(image->header.stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  try {
    if (profiler_ != nullptr) {
      profiler_->noteIngress(ingress_time, source_timestamp_us);
    }

    const GstVideoFormat format = gstFormatFromRosEncoding(image->encoding);
    if (format == GST_VIDEO_FORMAT_UNKNOWN) {
      throw std::runtime_error("Unsupported ROS image encoding '" + image->encoding + "'.");
    }

    const FrameLayout layout = frameLayoutFromImage(*image, format);
    // Raw appsrc caps are fixed when the pipeline starts. A frame-shape or
    // stride change is treated as a stream reconfiguration and forces rebuild.
    if (layout_) {
      const FrameLayout & previous_layout = *layout_;
      if (frameLayoutsMatch(previous_layout, layout)) {
        if (pipeline_ != nullptr) {
          pushLocked(*image);
          return;
        }
      } else {
        logFrameLayoutChange(spec_, previous_layout, layout);
      }
    }

    auto handles = takePipelineLocked();
    teardown(handles.pipeline, handles.appsrc, handles.appsink);
    startLocked(layout);
    pushLocked(*image);
  } catch (const std::exception & exc) {
    // Keep the ROS subscription alive after a frame-handling failure so the
    // next frame can rebuild the pipeline from the latest observed layout.
    observer_.onPushFailed(exc.what());
    auto handles = takePipelineLocked();
    teardown(handles.pipeline, handles.appsrc, handles.appsink);
  }
}

void RawRosVideoFrameSource::startLocked(const FrameLayout & layout)
{
  const char * format_name = gst_video_format_to_string(layout.format);
  if (format_name == nullptr) {
    throw std::runtime_error("Unsupported GStreamer video format.");
  }

  std::string ingress_fragment = "appsrc name=";
  ingress_fragment += kVideoAppSrcName;
  ingress_fragment += " is-live=true block=false format=time do-timestamp=true";
  ingress_fragment += " caps=video/x-raw,format=";
  ingress_fragment += format_name;
  ingress_fragment += ",width=";
  ingress_fragment += std::to_string(layout.width);
  ingress_fragment += ",height=";
  ingress_fragment += std::to_string(layout.height);
  ingress_fragment += ",framerate=0/1";
  startPipelineLocked(buildVideoPipelineDescription(ingress_fragment, spec_.transform_fragment), true);
  layout_ = layout;
}

void RawRosVideoFrameSource::pushLocked(const sensor_msgs::msg::Image & image)
{
  if (appsrc_ == nullptr) {
    throw std::runtime_error("Raw video appsrc is unavailable.");
  }
  if (!layout_) {
    throw std::runtime_error("Raw video layout is unavailable.");
  }

  VideoStreamProfiler::StageTimer push_timer(
    profiler_.get(), VideoProfileStage::kPushToAppSrc, timestampUsFromRosStamp(image.header.stamp));
  GstBufferPtr buffer = makeStampedGstBuffer(image.data.data(), image.data.size(), image.header.stamp);
  const FrameLayout & layout = *layout_;

  // Preserve the ROS row stride for downstream elements instead of assuming the
  // image bytes are tightly packed for this format.
  gsize offsets[GST_VIDEO_MAX_PLANES] = {0};
  gint strides[GST_VIDEO_MAX_PLANES] = {static_cast<gint>(layout.stride)};
  (void)gst_buffer_add_video_meta_full(
    buffer.get(),
    GST_VIDEO_FRAME_FLAG_NONE,
    layout.format,
    static_cast<guint>(layout.width),
    static_cast<guint>(layout.height),
    1,
    offsets,
    strides);

  if (gst_app_src_push_buffer(appsrc_.get(), buffer.release()) != GST_FLOW_OK) {
    throw std::runtime_error("Failed to push raw ROS image into GStreamer.");
  }
}

void RawRosVideoFrameSource::resetLocked()
{
  layout_.reset();
}

CompressedRosVideoFrameSource::CompressedRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: VideoPipelineFrameSource(std::move(spec), sink, observer, std::move(profiler))
, node_(node)
, qos_config_(qos_config)
{}

void CompressedRosVideoFrameSource::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  if (subscription_) {
    return;
  }

  const rclcpp::QoS base_qos(rclcpp::KeepLast(1));
  ResolvedSubscriptionQos qos = resolveSubscriptionQos(node_, spec_.ros_topic, base_qos, qos_config_);
  logSubscriptionQos(spec_, qos);

  // The ROS subscription may still have queued callbacks during shutdown; the
  // weak ref prevents that queue from extending this source's lifetime.
  auto weak_self = std::weak_ptr<CompressedRosVideoFrameSource>(
    std::static_pointer_cast<CompressedRosVideoFrameSource>(shared_from_this()));
  subscription_ = node_.create_subscription<sensor_msgs::msg::CompressedImage>(
    spec_.ros_topic, qos.qos, [weak_self](const sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
      if (const auto self = weak_self.lock(); self) {
        self->onImage(image);
      }
    });
}

void CompressedRosVideoFrameSource::shutdown()
{
  PipelineHandles handles;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    subscription = std::move(subscription_);
    handles = takePipelineLocked();
  }

  // Release ROS/GStreamer objects after unlocking so any destruction-triggered
  // callbacks never re-enter this source while mutex_ is held.
  subscription.reset();
  teardown(handles.pipeline, handles.appsrc, handles.appsink);
}

void CompressedRosVideoFrameSource::onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image)
{
  const auto ingress_time = VideoStreamProfiler::SteadyClock::now();
  const auto source_timestamp_us = timestampUsFromRosStamp(image->header.stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  try {
    if (profiler_ != nullptr) {
      profiler_->noteIngress(ingress_time, source_timestamp_us);
    }

    const CompressedImageCodec codec = compressedImageCodecFromImage(*image);

    // The decoder chain differs per codec (`jpegdec` vs `pngdec`), so the
    // pipeline is recreated whenever the advertised codec changes.
    if (codec_) {
      if (*codec_ == codec) {
        if (pipeline_ != nullptr) {
          pushLocked(*image);
          return;
        }
      } else {
        LogEvent(kLogger, "video_stream_input_codec_changed")
          .field("stream_key", spec_.stream_key)
          .field("topic", spec_.ros_topic)
          .field("previous_codec", compressedImageCodecName(*codec_))
          .field("codec", compressedImageCodecName(codec))
          .info();
      }
    }

    auto handles = takePipelineLocked();
    teardown(handles.pipeline, handles.appsrc, handles.appsink);
    startLocked(codec);
    pushLocked(*image);
  } catch (const std::exception & exc) {
    // Keep the ROS subscription alive after a frame-handling failure so the
    // next frame can rebuild the decoder chain from the next advertised codec.
    observer_.onPushFailed(exc.what());
    auto handles = takePipelineLocked();
    teardown(handles.pipeline, handles.appsrc, handles.appsink);
  }
}

void CompressedRosVideoFrameSource::startLocked(CompressedImageCodec codec)
{
  std::string ingress_fragment = "appsrc name=";
  ingress_fragment += kVideoAppSrcName;
  ingress_fragment += " is-live=true block=false format=time do-timestamp=true";
  switch (codec) {
    case CompressedImageCodec::kJpeg:
      ingress_fragment += " caps=image/jpeg ! jpegdec";
      break;
    case CompressedImageCodec::kPng:
      ingress_fragment += " caps=image/png ! pngdec";
      break;
    default:
      throw std::logic_error("Unsupported compressed image codec.");
  }
  startPipelineLocked(buildVideoPipelineDescription(ingress_fragment, spec_.transform_fragment), true);
  codec_ = codec;
}

void CompressedRosVideoFrameSource::pushLocked(const sensor_msgs::msg::CompressedImage & image)
{
  if (appsrc_ == nullptr) {
    throw std::runtime_error("Compressed video appsrc is unavailable.");
  }

  VideoStreamProfiler::StageTimer push_timer(
    profiler_.get(), VideoProfileStage::kPushToAppSrc, timestampUsFromRosStamp(image.header.stamp));
  GstBufferPtr buffer = makeStampedGstBuffer(image.data.data(), image.data.size(), image.header.stamp);
  if (gst_app_src_push_buffer(appsrc_.get(), buffer.release()) != GST_FLOW_OK) {
    throw std::runtime_error("Failed to push compressed ROS image into GStreamer.");
  }
}

void CompressedRosVideoFrameSource::resetLocked()
{
  codec_.reset();
}

}  // namespace livekit_ros2_bridge
