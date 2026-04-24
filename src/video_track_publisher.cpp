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
#include <stdexcept>
#include <utility>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "video_frame_source/ros_topic_video_frame_sources.hpp"
#include "video_frame_source/video_pipeline_frame_source.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");

void unpublishVideoTrackBestEffort(
  RoomConnection & room_connection, const std::shared_ptr<livekit::LocalVideoTrack> & track) noexcept
{
  if (track == nullptr) {
    return;
  }

  try {
    room_connection.unpublishVideoTrack(track);
  } catch (...) {}
}

std::shared_ptr<VideoFrameSource> makeVideoFrameSource(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  const VideoStreamSpec & spec,
  const SubscriptionQosConfig * qos_config,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer)
{
  if (otherVideoInput(spec) != nullptr) {
    return makeOtherVideoFrameSource(spec, sink, observer);
  }

  auto source = std::make_shared<RosTopicVideoFrameSource>(
    std::move(parameters), std::move(topics), std::move(graph), spec, qos_config, sink, observer);
  source->activate();
  return source;
}

}  // namespace

std::shared_ptr<VideoTrackPublisher> VideoTrackPublisher::create(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config)
{
  auto publisher = std::shared_ptr<VideoTrackPublisher>(new VideoTrackPublisher(room_connection, std::move(spec)));
  publisher->frame_source_ = makeVideoFrameSource(
    std::move(parameters), std::move(topics), std::move(graph), publisher->spec_, qos_config, *publisher, *publisher);
  return publisher;
}

VideoTrackPublisher::VideoTrackPublisher(RoomConnection & room_connection, VideoStreamSpec spec)
: room_connection_(room_connection)
, spec_(std::move(spec))
{}

VideoTrackPublisher::~VideoTrackPublisher()
{
  close();
}

void VideoTrackPublisher::captureFrame(const livekit::VideoFrame & frame, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  const int width = frame.width();
  const int height = frame.height();
  if (video_source_ == nullptr || video_source_->width() != width || video_source_->height() != height) {
    const bool is_republish = was_published_;

    try {
      unpublishVideoTrackBestEffort(room_connection_, published_video_track_);
      published_video_track_.reset();
      video_source_.reset();
      auto video_source = std::make_shared<livekit::VideoSource>(width, height);
      auto published_video_track =
        room_connection_.publishVideoTrack(spec_.track_name, video_source, spec_.publish_config);
      video_source_ = std::move(video_source);
      published_video_track_ = std::move(published_video_track);
    } catch (...) {
      LogEvent(kLogger, "video_track_publish_failed")
        .field("track_name", spec_.track_name)
        .field("width", width)
        .field("height", height)
        .fieldIfNotEmpty("stage", is_republish ? "republish_publish" : nullptr)
        .fieldException("error", std::current_exception())
        .warn();
      throw;
    }

    was_published_ = true;
    LogEvent(kLogger, is_republish ? "video_stream_track_republished" : "video_stream_track_published")
      .field("stream_key", spec_.stream_key)
      .field("width", width)
      .field("height", height)
      .info();
  }

  video_source_->captureFrame(frame, timestamp_us);
}

void VideoTrackPublisher::close()
{
  std::shared_ptr<VideoFrameSource> frame_source;
  std::shared_ptr<livekit::VideoSource> video_source;
  std::shared_ptr<livekit::LocalVideoTrack> published_video_track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_closed_) {
      return;
    }

    if (was_published_) {
      LogEvent(kLogger, "video_stream_track_unpublishing").field("stream_key", spec_.stream_key).info();
      was_published_ = false;
    }
    is_closed_ = true;
    frame_source = std::move(frame_source_);
    video_source = std::move(video_source_);
    published_video_track = std::move(published_video_track_);
  }

  if (frame_source != nullptr) {
    frame_source->close();
  }
  unpublishVideoTrackBestEffort(room_connection_, published_video_track);
  published_video_track.reset();
  video_source.reset();
}

void VideoTrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
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
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onRestartFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onPushFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

}  // namespace livekit_ros2_bridge
