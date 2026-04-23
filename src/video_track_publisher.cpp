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
#include <optional>
#include <stdexcept>
#include <utility>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "video_frame_source/other_video_frame_source.hpp"
#include "video_frame_source/ros_topic_video_frame_sources.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");

std::shared_ptr<VideoFrameSource> makeVideoFrameSource(
  SubscriptionNodeInterfaces interfaces,
  const VideoStreamSpec & spec,
  const SubscriptionQosConfig * qos_config,
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  const std::shared_ptr<VideoStreamProfiler> & profiler)
{
  if (spec.input_kind == VideoInputKind::OtherVideoSource) {
    auto source = std::make_shared<OtherVideoFrameSource>(spec, sink, observer, profiler);
    source->activate();
    return source;
  }
  if (spec.input_kind == VideoInputKind::RosTopic) {
    if (spec.ingest_mode == kRawImageIngestMode) {
      auto source =
        std::make_shared<RawRosVideoFrameSource>(std::move(interfaces), spec, qos_config, sink, observer, profiler);
      source->activate();
      return source;
    }
    if (spec.ingest_mode == kCompressedImageIngestMode) {
      auto source = std::make_shared<CompressedRosVideoFrameSource>(
        std::move(interfaces), spec, qos_config, sink, observer, profiler);
      source->activate();
      return source;
    }
  }

  LogEvent(kLogger, "video_stream_activate_failed")
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

class VideoTrackPublisher::Publication final
{
public:
  Publication(RoomConnection & room_connection, const VideoStreamSpec & spec, int width, int height)
  : room_connection_(room_connection)
  , width_(width)
  , height_(height)
  , video_source_(std::make_shared<livekit::VideoSource>(width, height))
  , track_(room_connection_.publishVideoTrack(spec.track_name, video_source_, spec.publish_config))
  {}

  ~Publication()
  {
    if (track_ == nullptr) {
      return;
    }

    try {
      room_connection_.unpublishVideoTrack(track_);
    } catch (...) {
      LogEvent(kLogger, "video_track_unpublish_failed")
        .field("track_name", track_->name)
        .fieldException("error", std::current_exception())
        .warn();
    }
  }

  Publication(const Publication &) = delete;
  Publication & operator=(const Publication &) = delete;
  Publication(Publication &&) = delete;
  Publication & operator=(Publication &&) = delete;

  bool matches(int width, int height) const noexcept
  {
    return width_ == width && height_ == height;
  }

  void capture(livekit::VideoFrame & frame, std::int64_t timestamp_us) const
  {
    video_source_->captureFrame(frame, timestamp_us);
  }

private:
  RoomConnection & room_connection_;
  int width_;
  int height_;
  std::shared_ptr<livekit::VideoSource> video_source_;
  std::shared_ptr<VideoTrackHandle> track_;
};

std::shared_ptr<VideoTrackPublisher> VideoTrackPublisher::create(
  SubscriptionNodeInterfaces interfaces,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * qos_config,
  std::shared_ptr<VideoStreamProfiler> profiler)
{
  auto publisher = std::shared_ptr<VideoTrackPublisher>(
    new VideoTrackPublisher(room_connection, std::move(spec), std::move(profiler)));
  publisher->frame_source_ = makeVideoFrameSource(
    std::move(interfaces), publisher->spec_, qos_config, *publisher, *publisher, publisher->profiler_);
  return publisher;
}

VideoTrackPublisher::VideoTrackPublisher(
  RoomConnection & room_connection, VideoStreamSpec spec, std::shared_ptr<VideoStreamProfiler> profiler)
: room_connection_(room_connection)
, spec_(std::move(spec))
, profiler_(std::move(profiler))
{}

VideoTrackPublisher::~VideoTrackPublisher()
{
  close();
}

void VideoTrackPublisher::write(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  const std::optional<std::int64_t> timestamp_us_opt =
    timestamp_us > 0 ? std::optional<std::int64_t>(timestamp_us) : std::nullopt;
  VideoStreamProfiler::StageTimer handle_timer(
    profiler_.get(), VideoProfileStage::kPublisherHandleFrame, timestamp_us_opt);

  if (publication_ == nullptr || !publication_->matches(width, height)) {
    const bool is_republish = was_published_;
    publication_.reset();

    try {
      publication_ = std::make_unique<Publication>(room_connection_, spec_, width, height);
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
    if (profiler_ != nullptr) {
      profiler_->noteTrackPublished(width, height, is_republish);
    }
    LogEvent(kLogger, is_republish ? "video_stream_track_republished" : "video_stream_track_published")
      .field("stream_key", spec_.stream_key)
      .field("width", width)
      .field("height", height)
      .info();
  }

  livekit::VideoFrame frame(width, height, livekit::VideoBufferType::I420, std::move(i420));
  {
    VideoStreamProfiler::StageTimer capture_timer(profiler_.get(), VideoProfileStage::kCaptureFrame, timestamp_us_opt);
    publication_->capture(frame, timestamp_us);
  }
  if (profiler_) {
    profiler_->noteCapture(timestamp_us_opt);
  }
}

void VideoTrackPublisher::close()
{
  std::shared_ptr<VideoFrameSource> frame_source;
  std::unique_ptr<Publication> publication;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_closed_) {
      return;
    }

    if (was_published_) {
      if (profiler_ != nullptr) {
        profiler_->noteTrackUnpublish();
      }
      LogEvent(kLogger, "video_stream_track_unpublishing").field("stream_key", spec_.stream_key).info();
      was_published_ = false;
    }
    is_closed_ = true;
    frame_source = std::move(frame_source_);
    publication = std::move(publication_);
  }

  if (frame_source != nullptr) {
    frame_source->close();
  }
  publication.reset();
}

void VideoTrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->noteSampleUnpackFailed(error);
  }

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
  if (profiler_ != nullptr) {
    profiler_->noteCaptureFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onPipelineFailed(const std::string & reason)
{
  if (profiler_ != nullptr) {
    profiler_->notePipelineFailure(reason);
  }
}

void VideoTrackPublisher::onRestartFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->noteRestartFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

void VideoTrackPublisher::onPushFailed(const std::string & error)
{
  if (profiler_ != nullptr) {
    profiler_->notePushFailed(error);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed").field("stream_key", spec_.stream_key).field("error", error).warn();
}

}  // namespace livekit_ros2_bridge
