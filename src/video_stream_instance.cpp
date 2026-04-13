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
#include "video_frame_source/configured_source_video_frame_source.hpp"
#include "video_frame_source/ros_topic_video_frame_sources.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_stream_instance");

}  // namespace

VideoStreamInstance::VideoStreamInstance(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config,
  std::shared_ptr<VideoStreamProfiler> profiler)
: node_(node)
, spec_(std::move(spec))
, qos_config_(qos_config)
, profiler_(std::move(profiler))
, publisher_(std::make_unique<VideoTrackPublisher>(room_connection, spec_, *this, profiler_))
{}

VideoStreamInstance::~VideoStreamInstance()
{
  shutdown();
}

void VideoStreamInstance::onTrackPublished(int width, int height, bool republished)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  published_dimensions_ = TrackDimensions{width, height};
  if (profiler_ != nullptr) {
    profiler_->noteTrackPublished(width, height, republished);
  }
  LogEvent(kLogger, republished ? "video_stream_track_republished" : "video_stream_track_published")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("width", width)
    .field("height", height)
    .info();
}

void VideoStreamInstance::onTrackUnpublishing()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!published_dimensions_) {
    return;
  }

  LogEvent(kLogger, "video_stream_track_unpublishing")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("width", published_dimensions_->width)
    .field("height", published_dimensions_->height)
    .info();
  if (profiler_ != nullptr) {
    profiler_->noteTrackUnpublish();
  }
  published_dimensions_.reset();
}

void VideoStreamInstance::onSampleUnpackFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->noteSampleUnpackFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  LogEvent(kLogger, "video_stream_sample_unpack_failed")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", error)
    .warn();
}

void VideoStreamInstance::onCaptureFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->noteCaptureFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", error)
    .warn();
}

void VideoStreamInstance::onPipelineFailed(const std::string & reason)
{
  if (profiler_ != nullptr) {
    profiler_->notePipelineFailure(reason);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  LogEvent(kLogger, "video_stream_pipeline_failed")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", reason)
    .warn();
}

void VideoStreamInstance::onRestartFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->noteRestartFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", error)
    .warn();
}

void VideoStreamInstance::onPushFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->notePushFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("error", error)
    .warn();
}

std::string VideoStreamInstance::start()
{
  std::shared_ptr<VideoFrameSource> source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream is shut down.");
    }

    if (!source_) {
      if (spec_.input_kind == VideoInputKind::ConfiguredSource) {
        source_ = std::make_shared<ConfiguredSourceVideoFrameSource>(spec_, *publisher_, *this, profiler_);
      } else if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kRawImageIngestMode) {
        source_ = std::make_shared<RawRosVideoFrameSource>(node_, spec_, qos_config_, *publisher_, *this, profiler_);
      } else if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kCompressedImageIngestMode) {
        source_ =
          std::make_shared<CompressedRosVideoFrameSource>(node_, spec_, qos_config_, *publisher_, *this, profiler_);
      } else {
        throw std::runtime_error(
          "Unsupported video input kind/ingest mode combination '" + videoInputKindToString(spec_.input_kind) + "/" +
          spec_.ingest_mode + "'.");
      }
    }
    // Hold a shared ref across the unlocked start() call so concurrent shutdown can
    // detach source_ without destroying the source underneath this invocation.
    source = source_;
  }

  source->start();
  return spec_.track_name;
}

void VideoStreamInstance::shutdown()
{
  std::shared_ptr<VideoFrameSource> source;
  std::unique_ptr<VideoTrackPublisher> publisher;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    // Mark shutdown and drop owned runtime handles under mutex_ so their teardown runs
    // after the lock is released.
    source = std::move(source_);
    publisher = std::move(publisher_);
  }

  // Stop ingress first so no new frames race into publisher teardown/unpublish.
  if (source) {
    source->shutdown();
  }
  if (publisher) {
    publisher->shutdown();
  }
}

}  // namespace livekit_ros2_bridge
