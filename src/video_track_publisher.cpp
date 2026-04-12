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

#include <utility>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"

namespace livekit_ros2_bridge
{
VideoTrackPublisher::VideoTrackPublisher(
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler)
: room_connection_(room_connection)
, spec_(std::move(spec))
, lifecycle_observer_(lifecycle_observer)
, profiler_(std::move(profiler))
{}

void VideoTrackPublisher::handleFrame(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  const std::optional<std::int64_t> frame_timestamp_us =
    timestamp_us > 0 ? std::optional<std::int64_t>(timestamp_us) : std::nullopt;
  VideoStreamProfiler::ScopedStageTimer publisher_handle_timer(
    profiler_.get(), VideoProfileStage::kPublisherHandleFrame, frame_timestamp_us);
  {
    VideoStreamProfiler::ScopedStageTimer ensure_track_timer(
      profiler_.get(), VideoProfileStage::kEnsureTrack, frame_timestamp_us);
    ensurePublishedTrackLocked(width, height);
  }
  // The public LiveKit C++ SDK takes an owned VideoFrame buffer here, so this
  // remains a low-copy path rather than true zero-copy. Keeping the data in
  // I420 still avoids the more expensive per-frame color conversion.
  livekit::VideoFrame frame(width, height, livekit::VideoBufferType::I420, std::move(i420));
  {
    VideoStreamProfiler::ScopedStageTimer capture_frame_timer(
      profiler_.get(), VideoProfileStage::kCaptureFrame, frame_timestamp_us);
    video_source_->captureFrame(frame, timestamp_us);
  }
  if (profiler_ != nullptr) {
    profiler_->noteFrameCaptured(frame_timestamp_us);
  }
}

void VideoTrackPublisher::shutdown()
{
  std::shared_ptr<PublishedVideoTrack> published_track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    published_track = std::move(published_track_);
    video_source_.reset();
    published_width_ = 0;
    published_height_ = 0;
  }

  if (published_track) {
    lifecycle_observer_.onVideoTrackUnpublishing();
    room_connection_.unpublishVideoTrack(published_track);
  }
}

void VideoTrackPublisher::ensurePublishedTrackLocked(int width, int height)
{
  if (
    video_source_ != nullptr && published_track_ != nullptr && published_width_ == width && published_height_ == height)
  {
    return;
  }

  const bool republishing = published_track_ != nullptr;
  if (published_track_) {
    room_connection_.unpublishVideoTrack(published_track_);
    published_track_.reset();
  }

  video_source_ = std::make_shared<livekit::VideoSource>(width, height);
  published_track_ = room_connection_.publishVideoTrack(spec_.track_name, video_source_, spec_.publish_config);
  published_width_ = width;
  published_height_ = height;
  lifecycle_observer_.onVideoTrackPublished(width, height, republishing);
}

}  // namespace livekit_ros2_bridge
