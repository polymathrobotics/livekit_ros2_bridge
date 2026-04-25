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

#include "video_track_publisher.hpp"

#include <exception>
#include <memory>
#include <utility>
#include <variant>

#include "gstreamer_video_stream.hpp"
#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "ros_video_stream.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");

void unpublishBestEffort(RoomConnection & connection, const std::shared_ptr<livekit::LocalVideoTrack> & track) noexcept
{
  if (track == nullptr) {
    return;
  }

  try {
    connection.unpublishVideoTrack(track);
  } catch (...) {}
}

}  // namespace

std::shared_ptr<VideoTrackPublisher> VideoTrackPublisher::create(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config)
{
  auto publisher = std::make_shared<VideoTrackPublisher>(room_connection, std::move(spec));
  if (std::holds_alternative<OtherVideoInput>(publisher->spec_.input)) {
    auto stream = std::make_unique<GStreamerVideoStream>(publisher->spec_, *publisher);
    stream->start();
    publisher->gstreamer_stream_ = std::move(stream);
    return publisher;
  }

  auto stream = std::make_shared<RosVideoStream>(std::move(node_interfaces), publisher->spec_, qos_config, *publisher);
  stream->start();
  publisher->ros_stream_ = std::move(stream);
  return publisher;
}

VideoTrackPublisher::VideoTrackPublisher(RoomConnection & room_connection, VideoStreamSpec spec)
: connection_(room_connection)
, spec_(std::move(spec))
{}

VideoTrackPublisher::~VideoTrackPublisher()
{
  close();
}

void VideoTrackPublisher::captureFrame(const livekit::VideoFrame & frame, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  const int width = frame.width();
  const int height = frame.height();
  if (source_ == nullptr || source_->width() != width || source_->height() != height) {
    const bool republish = published_once_;

    try {
      unpublishBestEffort(connection_, track_);
      track_.reset();
      source_.reset();
      auto source = std::make_shared<livekit::VideoSource>(width, height);
      auto track = connection_.publishVideoTrack(spec_.track_name, source, spec_.publish_options);
      source_ = std::move(source);
      track_ = std::move(track);
    } catch (...) {
      LogEvent(kLogger, republish ? "video_track_republish_failed" : "video_track_publish_failed")
        .field("track_name", spec_.track_name)
        .field("width", width)
        .field("height", height)
        .fieldException("error", std::current_exception())
        .warn();
      throw;
    }

    published_once_ = true;
    if (republish) {
      LogEvent(kLogger, "video_stream_track_republished")
        .field("stream_key", spec_.stream_key)
        .field("width", width)
        .field("height", height)
        .info();
    }
  }

  source_->captureFrame(frame, timestamp_us);
}

void VideoTrackPublisher::close()
{
  // Stop streams outside mutex_; their callbacks can re-enter this publisher.
  std::shared_ptr<RosVideoStream> ros_stream;
  std::unique_ptr<GStreamerVideoStream> gstreamer_stream;
  std::shared_ptr<livekit::VideoSource> source;
  std::shared_ptr<livekit::LocalVideoTrack> track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    published_once_ = false;
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
  unpublishBestEffort(connection_, track);
  track.reset();
  source.reset();
}

void VideoTrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_sample_unpack_failed")
    .field("stream_key", spec_.stream_key)
    .field("error", error)
    .warn();
}

void VideoTrackPublisher::onCaptureFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onRestartFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onPushFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

}  // namespace livekit_ros2_bridge
