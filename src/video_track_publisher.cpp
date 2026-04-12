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
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamRegistryLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

}  // namespace

VideoTrackPublisher::VideoTrackPublisher(RoomSession & session, VideoStreamSpec spec)
: session_(session)
, spec_(std::move(spec))
{}

void VideoTrackPublisher::handleFrame(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  ensurePublishedTrackLocked(width, height);
  // The public LiveKit C++ SDK takes an owned VideoFrame buffer here, so this
  // remains a low-copy path rather than true zero-copy. Keeping the data in
  // I420 still avoids the more expensive per-frame color conversion.
  livekit::VideoFrame frame(width, height, livekit::VideoBufferType::I420, std::move(i420));
  video_source_->captureFrame(frame, timestamp_us);
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
    LogEvent(kVideoStreamRegistryLogger, "video_stream_track_unpublishing")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .info();
    session_.unpublishVideoTrack(published_track);
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
    LogEvent(kVideoStreamRegistryLogger, "video_stream_track_replacing")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("previous_width", published_width_)
      .field("previous_height", published_height_)
      .field("next_width", width)
      .field("next_height", height)
      .info();
    session_.unpublishVideoTrack(published_track_);
    published_track_.reset();
  }

  video_source_ = std::make_shared<livekit::VideoSource>(width, height);
  published_track_ = session_.publishVideoTrack(spec_.track_name, video_source_, spec_.publish_config);
  published_width_ = width;
  published_height_ = height;

  LogEvent(kVideoStreamRegistryLogger, republishing ? "video_stream_track_republished" : "video_stream_track_published")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("width", width)
    .field("height", height)
    .info();
}

}  // namespace livekit_ros2_bridge
