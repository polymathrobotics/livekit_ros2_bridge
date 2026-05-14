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

#include "video/track_publisher.hpp"

#include <cstdint>
#include <memory>
#include <utility>
#include <variant>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/gstreamer_stream.hpp"
#include "video/ros_stream.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");

void tryUnpublish(RoomConnection & connection, const std::shared_ptr<livekit::LocalVideoTrack> & track) noexcept
{
  if (track == nullptr) {
    return;
  }

  try {
    connection.unpublishVideoTrack(track);
  } catch (...) {}
}

livekit::TrackPublishOptions publishOptionsWithFrameMetadata(livekit::TrackPublishOptions options)
{
  options.packet_trailer_features.user_timestamp = true;
  return options;
}

livekit::VideoCaptureOptions captureOptions(std::int64_t timestamp_us)
{
  livekit::VideoCaptureOptions options;
  options.timestamp_us = timestamp_us;
  if (timestamp_us >= 0) {
    livekit::VideoFrameMetadata metadata;
    metadata.user_timestamp_us = static_cast<std::uint64_t>(timestamp_us);
    options.metadata = metadata;
  }
  return options;
}

}  // namespace

std::shared_ptr<TrackPublisher> TrackPublisher::create(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
  RoomConnection & connection,
  StreamSpec spec,
  const SubscriptionQosConfig * qos_config)
{
  auto publisher = std::make_shared<TrackPublisher>(connection, std::move(spec));
  if (std::holds_alternative<OtherInput>(publisher->spec_.input)) {
    auto stream = std::make_unique<GStreamerStream>(publisher->spec_, *publisher);
    stream->start();
    publisher->gstreamer_stream_ = std::move(stream);
    return publisher;
  }

  auto stream = std::make_shared<RosStream>(std::move(node_interfaces), publisher->spec_, qos_config, *publisher);
  stream->start();
  publisher->ros_stream_ = std::move(stream);
  return publisher;
}

TrackPublisher::TrackPublisher(RoomConnection & connection, StreamSpec spec)
: connection_(connection)
, spec_(std::move(spec))
{}

TrackPublisher::~TrackPublisher()
{
  close();
}

PipelineCallbacks TrackPublisher::makePipelineCallbacks(
  std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed)
{
  return PipelineCallbacks{
    std::move(is_shutdown),
    [this](const livekit::VideoFrame & frame, std::int64_t timestamp_us) { capture(frame, timestamp_us); },
    [this](const std::string & error) { onSampleUnpackFailed(error); },
    [this](const std::string & error) { onCaptureFailed(error); },
    std::move(on_failed),
  };
}

void TrackPublisher::capture(const livekit::VideoFrame & frame, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  const int width = frame.width();
  const int height = frame.height();
  if (source_ == nullptr || source_->width() != width || source_->height() != height) {
    const bool republish = published_once_;

    tryUnpublish(connection_, track_);
    track_.reset();
    source_.reset();
    auto source = std::make_shared<livekit::VideoSource>(width, height);
    auto track =
      connection_.publishVideoTrack(spec_.track_name, source, publishOptionsWithFrameMetadata(spec_.publish_options));
    source_ = std::move(source);
    track_ = std::move(track);
    captured_frame_logged_ = false;

    published_once_ = true;
    if (republish) {
      LogEvent(kLogger, "video_stream_track_republished")
        .fieldOr("stream_key", spec_.stream_key)
        .field("width", width)
        .field("height", height)
        .info();
    }
  }

  source_->captureFrame(frame, captureOptions(timestamp_us));
  if (!captured_frame_logged_) {
    LogEvent(kLogger, "video_stream_frame_captured")
      .fieldOr("stream_key", spec_.stream_key)
      .fieldOr("track_name", spec_.track_name)
      .field("width", width)
      .field("height", height)
      .field("timestamp_us", timestamp_us)
      .info();
    captured_frame_logged_ = true;
  }
}

void TrackPublisher::close()
{
  // Stop streams outside mutex_; their callbacks can re-enter this publisher.
  std::shared_ptr<RosStream> ros_stream;
  std::unique_ptr<GStreamerStream> gstreamer_stream;
  std::shared_ptr<livekit::VideoSource> source;
  std::shared_ptr<livekit::LocalVideoTrack> track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    published_once_ = false;
    captured_frame_logged_ = false;
    closed_ = true;
    ros_stream = std::move(ros_stream_);
    gstreamer_stream = std::move(gstreamer_stream_);
    source = std::move(source_);
    track = std::move(track_);
  }

  if (ros_stream != nullptr) {
    ros_stream->close();
  }
  if (gstreamer_stream != nullptr) {
    gstreamer_stream->close();
  }
  tryUnpublish(connection_, track);
  track.reset();
  source.reset();
}

void TrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_sample_unpack_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::onCaptureFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::onRestartFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::onAppsrcPushFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed").fieldOr("stream_key", spec_.stream_key).fieldOr("error", error).warn();
}

}  // namespace livekit_ros2_bridge::video
