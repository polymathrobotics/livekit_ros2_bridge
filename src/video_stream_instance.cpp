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

std::shared_ptr<VideoFrameSource> makeVideoFrameSource(
  rclcpp::Node & node,
  const VideoStreamSpec & spec,
  const SubscriptionQosConfig * qos_config,
  VideoTrackPublisher & publisher,
  VideoStreamLifecycleObserver & observer,
  const std::shared_ptr<VideoStreamProfiler> & profiler);

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

std::string VideoStreamInstance::start()
{
  std::shared_ptr<VideoFrameSource> source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      LogEvent(kLogger, "video_stream_start_rejected")
        .field("stream_key", spec_.stream_key)
        .field("reason", "shutdown")
        .warn();
      throw std::runtime_error("Video stream is shut down.");
    }

    if (!source_) {
      source_ = makeVideoFrameSource(node_, spec_, qos_config_, *publisher_, *this, profiler_);
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

void VideoStreamInstance::onTrackPublished(int width, int height, bool republished)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  track_published_ = true;
  if (profiler_ != nullptr) {
    profiler_->noteTrackPublished(width, height, republished);
  }
  LogEvent(kLogger, republished ? "video_stream_track_republished" : "video_stream_track_published")
    .field("stream_key", spec_.stream_key)
    .field("width", width)
    .field("height", height)
    .info();
}

void VideoStreamInstance::onTrackUnpublish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!track_published_) {
    return;
  }

  if (profiler_ != nullptr) {
    profiler_->noteTrackUnpublish();
  }
  LogEvent(kLogger, "video_stream_track_unpublishing").field("stream_key", spec_.stream_key).info();
  track_published_ = false;
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

  LogEvent(kLogger, "video_stream_capture_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoStreamInstance::onPipelineFailed(const std::string & reason)
{
  if (profiler_ != nullptr) {
    profiler_->notePipelineFailure(reason);
  }
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

  LogEvent(kLogger, "video_stream_restart_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
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

  LogEvent(kLogger, "video_stream_push_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

namespace
{

std::shared_ptr<VideoFrameSource> makeVideoFrameSource(
  rclcpp::Node & node,
  const VideoStreamSpec & spec,
  const SubscriptionQosConfig * qos_config,
  VideoTrackPublisher & publisher,
  VideoStreamLifecycleObserver & observer,
  const std::shared_ptr<VideoStreamProfiler> & profiler)
{
  if (spec.input_kind == VideoInputKind::ConfiguredSource) {
    return std::make_shared<ConfiguredSourceVideoFrameSource>(spec, publisher, observer, profiler);
  }
  if (spec.input_kind == VideoInputKind::RosTopic) {
    if (spec.ingest_mode == kRawImageIngestMode) {
      return std::make_shared<RawRosVideoFrameSource>(node, spec, qos_config, publisher, observer, profiler);
    }
    if (spec.ingest_mode == kCompressedImageIngestMode) {
      return std::make_shared<CompressedRosVideoFrameSource>(node, spec, qos_config, publisher, observer, profiler);
    }
  }

  LogEvent(kLogger, "video_stream_start_failed")
    .field("stream_key", spec.stream_key)
    .field("input_kind", videoInputKindToString(spec.input_kind))
    .field("ingest_mode", spec.ingest_mode)
    .field("reason", "unsupported_input")
    .warn();
  throw std::runtime_error(
    "Unsupported video input kind/ingest mode combination '" + videoInputKindToString(spec.input_kind) + "/" +
    spec.ingest_mode + "'.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
