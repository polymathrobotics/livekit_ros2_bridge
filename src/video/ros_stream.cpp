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

#include "video/ros_stream.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "utils/log_event.hpp"
#include "utils/trim.hpp"
#include "video/pipeline_description.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.ros_video_stream");

GstVideoFormat formatFromRosEncoding(const std::string & encoding)
{
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

std::optional<std::string> parseCompressedCodec(const std::string & format)
{
  const auto parse_token = [](std::string token) -> std::optional<std::string> {
    token = trim(token);
    const auto token_end = token.find_first_of(" \t\r\n");
    if (token_end != std::string::npos) {
      token.resize(token_end);
    }
    std::transform(
      token.begin(), token.end(), token.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (token == "jpeg" || token == "jpg") return std::string{"jpeg"};
    if (token == "png") return std::string{"png"};
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

void logRawLayoutChange(const StreamSpec & spec, const RawLayout & previous, const RawLayout & layout)
{
  const auto & input = requireRosInput(spec);
  LogEvent event(kLogger, "video_stream_input_layout_changed");
  event.field("stream_key", spec.stream_key).field("topic", input.topic);
  if (previous.width != layout.width) {
    event.field("previous_width", previous.width).field("width", layout.width);
  }
  if (previous.height != layout.height) {
    event.field("previous_height", previous.height).field("height", layout.height);
  }
  if (previous.stride != layout.stride) {
    event.field("previous_stride", previous.stride).field("stride", layout.stride);
  }
  if (previous.format != layout.format) {
    const char * previous_format = gst_video_format_to_string(previous.format);
    const char * format = gst_video_format_to_string(layout.format);
    event.fieldOr("previous_format", previous_format).fieldOr("format", format);
  }
  event.info();
}

GstBufferPtr makeStampedBuffer(const std::uint8_t * data, std::size_t size, const builtin_interfaces::msg::Time & stamp)
{
  GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, size, nullptr));
  if (buffer == nullptr) {
    throw std::runtime_error("Failed to allocate GStreamer buffer.");
  }

  {
    GstBufferMap map(*buffer, GST_MAP_WRITE);
    if (!map.is_valid()) {
      throw std::runtime_error("Failed to map GStreamer buffer.");
    }
    if (size > 0U) {
      std::memcpy(map.get()->data, data, size);
    }
  }

  const GstClockTime pts = static_cast<GstClockTime>(rclcpp::Time(stamp).nanoseconds());
  GST_BUFFER_PTS(buffer.get()) = pts;
  GST_BUFFER_DTS(buffer.get()) = pts;
  GST_BUFFER_DURATION(buffer.get()) = GST_CLOCK_TIME_NONE;
  return buffer;
}

void logSubscriptionQos(const StreamSpec & spec, const ResolvedSubscriptionQos & qos)
{
  if (qos.source == SubscriptionQosResolutionSource::Fallback && !qos.mixed_reliability && !qos.mixed_durability) {
    return;
  }

  const auto & input = requireRosInput(spec);
  LogEvent(kLogger, "subscription_qos_resolved")
    .field("resource", input.topic)
    .field("interface_type", input.interface_type)
    .field("publisher_count", qos.publisher_count)
    .field("source", subscriptionQosSourceString(qos.source))
    .field("reliability", subscriptionQosReliabilityString(qos.qos.reliability()))
    .field("durability", subscriptionQosDurabilityString(qos.qos.durability()))
    .fieldIf(qos.mixed_reliability, "mixed_reliability", true)
    .fieldIf(qos.mixed_durability, "mixed_durability", true)
    .fieldIfNotEmpty("override_id", qos.override_id)
    .info();
}

}  // namespace

RosStream::RosStream(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
  StreamSpec spec,
  const SubscriptionQosConfig * qos_config,
  TrackPublisher & publisher)
: spec_(std::move(spec))
, publisher_(publisher)
, pipeline_(publisher.makePipelineCallbacks(
    [this]() { return isShutdown(); }, [this](const std::string & reason) { onPipelineFailure(reason); }))
, node_interfaces_(std::move(node_interfaces))
, qos_config_(qos_config)
, failure_handler_(std::chrono::milliseconds::zero(), [this]() { stopPipelineAfterFailure(); })
{}

RosStream::~RosStream()
{
  close();
}

void RosStream::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }
  if (raw_subscription_ || compressed_subscription_) {
    return;
  }

  const rclcpp::QoS base(rclcpp::KeepLast(1));
  const auto & input = requireRosInput(spec_);
  ResolvedSubscriptionQos qos =
    resolveSubscriptionQos(node_interfaces_.get_node_graph_interface(), input.topic, base, qos_config_);
  logSubscriptionQos(spec_, qos);

  std::weak_ptr<RosStream> weak = shared_from_this();
  switch (input.ingest_mode) {
    case RosIngestMode::RawImage:
      raw_subscription_ = rclcpp::create_subscription<sensor_msgs::msg::Image>(
        node_interfaces_, input.topic, qos.qos, [weak](const sensor_msgs::msg::Image::ConstSharedPtr image) {
          if (const auto self = weak.lock(); self) {
            self->onRawImage(image);
          }
        });
      return;
    case RosIngestMode::CompressedImage:
      compressed_subscription_ = rclcpp::create_subscription<sensor_msgs::msg::CompressedImage>(
        node_interfaces_, input.topic, qos.qos, [weak](const sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
          if (const auto self = weak.lock(); self) {
            self->onCompressedImage(image);
          }
        });
      return;
  }
}

void RosStream::close()
{
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_subscription;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    raw_subscription = std::move(raw_subscription_);
    compressed_subscription = std::move(compressed_subscription_);
  }

  failure_handler_.close();

  // Subscription teardown can wait for executor callbacks, which take mutex_.
  raw_subscription.reset();
  compressed_subscription.reset();
  pipeline_.stop();
}

bool RosStream::isShutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

void RosStream::onPipelineFailure(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || !pipeline_.isActive()) {
    return;
  }
  if (!failure_handler_.schedule()) {
    return;
  }

  LogEvent(kLogger, "video_stream_pipeline_recovery_disabled")
    .field("stream_key", spec_.stream_key)
    .field("reason", reason)
    .warn();
}

void RosStream::stopPipelineAfterFailure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  resetPipelineStateLocked();
  pipeline_.stop();
}

void RosStream::onRawImage(const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  try {
    const GstVideoFormat format = formatFromRosEncoding(image->encoding);
    if (format == GST_VIDEO_FORMAT_UNKNOWN) {
      throw std::runtime_error("Unsupported ROS image encoding '" + image->encoding + "'.");
    }

    RawLayout layout;
    layout.width = static_cast<int>(image->width);
    layout.height = static_cast<int>(image->height);
    layout.format = format;
    layout.stride = image->step;
    if (raw_layout_) {
      const RawLayout & previous = *raw_layout_;
      if (
        previous.width != layout.width || previous.height != layout.height || previous.format != layout.format ||
        previous.stride != layout.stride)
      {
        logRawLayoutChange(spec_, previous, layout);
      } else if (pipeline_.isActive()) {
        pushRawLocked(*image);
        return;
      }
    }

    resetPipelineStateLocked();
    pipeline_.stop();
    startRawPipelineLocked(layout);
    pushRawLocked(*image);
  } catch (const std::exception & exc) {
    publisher_.onAppsrcPushFailed(exc.what());
    resetPipelineStateLocked();
    pipeline_.stop();
  }
}

void RosStream::startRawPipelineLocked(const RawLayout & layout)
{
  const char * format = gst_video_format_to_string(layout.format);

  std::string appsrc = "appsrc name=";
  appsrc += kBridgeAppSrcName;
  appsrc += " is-live=true block=false format=time do-timestamp=true";
  appsrc += " caps=video/x-raw,format=";
  appsrc += format;
  appsrc += ",width=";
  appsrc += std::to_string(layout.width);
  appsrc += ",height=";
  appsrc += std::to_string(layout.height);
  appsrc += ",framerate=0/1";
  pipeline_.start(buildPipelineDescription(appsrc, requireRosInput(spec_).transform_fragment), true);
  raw_layout_ = layout;
}

void RosStream::pushRawLocked(const sensor_msgs::msg::Image & image)
{
  GstBufferPtr buffer = makeStampedBuffer(image.data.data(), image.data.size(), image.header.stamp);
  const RawLayout & layout = *raw_layout_;

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

  if (gst_app_src_push_buffer(pipeline_.appsrc(), buffer.release()) != GST_FLOW_OK) {
    throw std::runtime_error("Failed to push raw ROS image into GStreamer.");
  }
}

void RosStream::onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  try {
    const auto parsed = parseCompressedCodec(image->format);
    if (!parsed.has_value()) {
      throw std::runtime_error("Unsupported compressed image format '" + image->format + "'.");
    }
    const std::string codec = *parsed;
    bool should_start = true;
    if (codec_) {
      if (*codec_ != codec) {
        const auto & input = requireRosInput(spec_);
        LogEvent event(kLogger, "video_stream_input_codec_changed");
        event.field("stream_key", spec_.stream_key).field("topic", input.topic);
        event.field("previous_codec", *codec_).field("codec", codec).info();
      } else if (pipeline_.isActive()) {
        should_start = false;
      }
    }

    if (should_start) {
      resetPipelineStateLocked();
      pipeline_.stop();
      startCompressedPipelineLocked(codec);
    }

    GstBufferPtr buffer = makeStampedBuffer(image->data.data(), image->data.size(), image->header.stamp);
    if (gst_app_src_push_buffer(pipeline_.appsrc(), buffer.release()) != GST_FLOW_OK) {
      throw std::runtime_error("Failed to push compressed ROS image into GStreamer.");
    }
  } catch (const std::exception & exc) {
    publisher_.onAppsrcPushFailed(exc.what());
    resetPipelineStateLocked();
    pipeline_.stop();
  }
}

void RosStream::startCompressedPipelineLocked(const std::string & codec)
{
  std::string appsrc = "appsrc name=";
  appsrc += kBridgeAppSrcName;
  appsrc += " is-live=true block=false format=time do-timestamp=true";
  appsrc += codec == "jpeg" ? " caps=image/jpeg ! jpegdec" : " caps=image/png ! pngdec";
  pipeline_.start(buildPipelineDescription(appsrc, requireRosInput(spec_).transform_fragment), true);
  codec_ = codec;
}

void RosStream::resetPipelineStateLocked()
{
  raw_layout_.reset();
  codec_.reset();
  failure_handler_.cancelPending();
}

}  // namespace livekit_ros2_bridge::video
