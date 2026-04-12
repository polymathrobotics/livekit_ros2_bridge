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

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "room_connection.hpp"
#include "video_stream_runtime.hpp"
#include "video_stream_spec.hpp"

namespace livekit
{
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

// Owned by a VideoStreamInstance. The paired VideoFrameSource handles input
// normalization and this publisher owns only one LiveKit video publication.
class VideoTrackPublisher final : public VideoFrameSink
{
public:
  VideoTrackPublisher(
    RoomConnection & room_connection, VideoStreamSpec spec, VideoStreamLifecycleObserver & lifecycle_observer);

  void handleFrame(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us) override;
  void shutdown();

private:
  void ensurePublishedTrackLocked(int width, int height);

  RoomConnection & room_connection_;
  VideoStreamSpec spec_;
  VideoStreamLifecycleObserver & lifecycle_observer_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  std::shared_ptr<livekit::VideoSource> video_source_;
  std::shared_ptr<PublishedVideoTrack> published_track_;
  int published_width_ = 0;
  int published_height_ = 0;
};

}  // namespace livekit_ros2_bridge
