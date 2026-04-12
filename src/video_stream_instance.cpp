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

#include "video_stream_instance.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "video_frame_source.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamInstanceLogger = rclcpp::get_logger("video_stream_instance");

}  // namespace

VideoStreamInstance::VideoStreamInstance(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, spec_(std::move(spec))
, subscription_qos_config_(subscription_qos_config)
, video_track_publisher_(std::make_unique<VideoTrackPublisher>(room_connection, spec_, *this))
{}

VideoStreamInstance::~VideoStreamInstance()
{
  shutdown();
}

void VideoStreamInstance::onVideoTrackPublished(int width, int height, bool republished)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  has_published_track_ = true;
  published_width_ = width;
  published_height_ = height;
  last_runtime_error_.clear();
  LogEvent(kVideoStreamInstanceLogger, republished ? "video_stream_track_republished" : "video_stream_track_published")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("width", width)
    .field("height", height)
    .info();
}

void VideoStreamInstance::onVideoTrackUnpublishing()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_published_track_) {
    return;
  }

  LogEvent(kVideoStreamInstanceLogger, "video_stream_track_unpublishing")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("width", published_width_)
    .field("height", published_height_)
    .info();
  has_published_track_ = false;
  published_width_ = 0;
  published_height_ = 0;
}

void VideoStreamInstance::onVideoStreamSampleUnpackFailed(const std::string & error)
{
  logRuntimeError("video_stream_sample_unpack_failed", error);
}

void VideoStreamInstance::onVideoStreamCaptureFailed(const std::string & error)
{
  logRuntimeError("video_stream_capture_failed", error);
}

void VideoStreamInstance::onVideoStreamPipelineFailed(const std::string & reason)
{
  logRuntimeError("video_stream_pipeline_failed", reason);
}

void VideoStreamInstance::onVideoStreamRestartFailed(const std::string & error)
{
  logRuntimeError("video_stream_restart_failed", error);
}

void VideoStreamInstance::onVideoStreamPushFailed(const std::string & error)
{
  logRuntimeError("video_stream_push_failed", error);
}

std::string VideoStreamInstance::ensureRunning()
{
  std::shared_ptr<VideoFrameSource> frame_source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (!frame_source_) {
      frame_source_ = createFrameSourceLocked();
    }
    frame_source = frame_source_;
  }

  frame_source->ensureRunning();
  return spec_.track_name;
}

void VideoStreamInstance::shutdown()
{
  std::shared_ptr<VideoFrameSource> frame_source;
  std::unique_ptr<VideoTrackPublisher> video_track_publisher;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    frame_source = std::move(frame_source_);
    video_track_publisher = std::move(video_track_publisher_);
  }

  if (frame_source) {
    frame_source->shutdown();
  }
  if (video_track_publisher) {
    video_track_publisher->shutdown();
  }
}

std::shared_ptr<VideoFrameSource> VideoStreamInstance::createFrameSourceLocked()
{
  if (spec_.input_kind == VideoInputKind::ConfiguredSource) {
    return makeConfiguredSourceVideoFrameSource(spec_, *video_track_publisher_, *this);
  }

  if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kRawImageIngestMode) {
    return makeRawRosVideoFrameSource(node_, spec_, subscription_qos_config_, *video_track_publisher_, *this);
  }
  if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kCompressedImageIngestMode) {
    return makeCompressedRosVideoFrameSource(node_, spec_, subscription_qos_config_, *video_track_publisher_, *this);
  }

  throw std::runtime_error(
    "Unsupported video input kind/ingest mode combination '" + videoInputKindToString(spec_.input_kind) + "/" +
    spec_.ingest_mode + "'.");
}

void VideoStreamInstance::logRuntimeError(const char * event_name, const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  last_runtime_error_ = error;
  LogEvent(kVideoStreamInstanceLogger, event_name)
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", error)
    .warn();
}

}  // namespace livekit_ros2_bridge
