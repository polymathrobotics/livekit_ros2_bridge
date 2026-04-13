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
#include <utility>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{
namespace
{
const auto kLogger = rclcpp::get_logger("video_track_publisher");
}  // namespace

VideoTrackPublisher::VideoTrackPublisher(
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  VideoStreamLifecycleObserver & observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: room_connection_(room_connection)
, spec_(std::move(spec))
, observer_(observer)
, profiler_(std::move(profiler))
{}

void VideoTrackPublisher::ensureTrack(int width, int height, const std::optional<std::int64_t> & timestamp_us)
{
  VideoStreamProfiler::StageTimer ensure_track_timer(profiler_.get(), VideoProfileStage::kEnsureTrack, timestamp_us);
  if (source_ != nullptr && track_ != nullptr && width_ == width && height_ == height) {
    return;
  }

  const bool republished = has_published_;
  const char * stage = track_ != nullptr ? "republish_unpublish" : nullptr;
  const auto logFailure = [&](std::exception_ptr exception = nullptr) {
    LogEvent(kLogger, "video_track_publish_failed")
      .field("track_name", spec_.track_name)
      .field("width", width)
      .field("height", height)
      .fieldIfNotEmpty("stage", stage)
      .fieldException("error", std::move(exception))
      .warn();
  };
  try {
    if (stage != nullptr) {
      room_connection_.unpublishVideoTrack(track_);
      stage = "republish_publish";
    }
    // Reset local publication state before publishing so a
    // publishVideoTrack() failure forces the next frame to retry instead of
    // reusing stale state.
    source_.reset();
    track_.reset();
    width_ = 0;
    height_ = 0;

    auto source = std::make_shared<livekit::VideoSource>(width, height);
    auto track = room_connection_.publishVideoTrack(spec_.track_name, source, spec_.publish_config);
    source_ = std::move(source);
    track_ = std::move(track);
    width_ = width;
    height_ = height;
    has_published_ = true;
    observer_.onTrackPublished(width, height, republished);
    // todo: is there another pattern we can use for this?
  } catch (...) {
    logFailure(std::current_exception());
    throw;
  }
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
  ensureTrack(width, height, timestamp_us_opt);
  // LiveKit takes ownership of the VideoFrame buffer here, so this stays
  // low-copy rather than true zero-copy. Keeping I420 avoids per-frame color conversion.
  livekit::VideoFrame frame(width, height, livekit::VideoBufferType::I420, std::move(i420));
  {
    VideoStreamProfiler::StageTimer capture_timer(profiler_.get(), VideoProfileStage::kCaptureFrame, timestamp_us_opt);
    source_->captureFrame(frame, timestamp_us);
  }
  if (profiler_) {
    profiler_->noteCapture(timestamp_us_opt);
  }
}

void VideoTrackPublisher::shutdown()
{
  std::shared_ptr<VideoTrackHandle> track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_closed_) {
      return;
    }

    is_closed_ = true;
    track = std::move(track_);
    source_.reset();
    width_ = 0;
    height_ = 0;
  }

  if (!track) {
    return;
  }

  // Run observer and RoomConnection callbacks after releasing mutex_. The
  // closed state is already visible, so concurrent or reentrant write() calls
  // will drop frames instead of racing a new publish against shutdown().
  observer_.onTrackUnpublish();
  try {
    room_connection_.unpublishVideoTrack(track);
  } catch (...) {
    LogEvent(kLogger, "video_track_unpublish_failed")
      .field("track_name", spec_.track_name)
      .fieldException("error", std::current_exception())
      .warn();
  }
}

}  // namespace livekit_ros2_bridge
