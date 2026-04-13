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
#include <optional>
#include <vector>

#include "room_connection.hpp"
#include "video_profiling.hpp"
#include "video_stream_runtime.hpp"
#include "video_stream_spec.hpp"

namespace livekit
{
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

// Owned by a VideoStreamInstance. The paired VideoFrameSource handles input
// normalization, and this publisher owns at most one LiveKit video publication
// at a time, replacing it when frame dimensions change.
class VideoTrackPublisher final : public VideoFrameSink
{
public:
  VideoTrackPublisher(
    RoomConnection & room_connection,
    VideoStreamSpec spec,
    VideoStreamLifecycleObserver & observer,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  // Thread-safe with shutdown(). The first successful frame lazily publishes the
  // track, and a resolution change tears down and recreates that publication.
  // Consumes the supplied I420 buffer because LiveKit takes ownership of the
  // frame payload during capture. Frames that arrive after shutdown() are
  // dropped.
  void write(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us) override;

  // Idempotent. Marks the publisher closed, waits for any in-flight write() to
  // leave the critical section, then unpublishes the current track if present.
  void shutdown();

private:
  void ensureTrackForFrame(int width, int height, const std::optional<std::int64_t> & timestamp_us_opt);

  RoomConnection & room_connection_;
  VideoStreamSpec spec_;
  // Callbacks run inline on whichever thread calls write()/shutdown(). write()
  // notifies only after the new publication state is committed under mutex_;
  // shutdown() notifies after closed state is visible and after releasing it.
  VideoStreamLifecycleObserver & observer_;
  std::shared_ptr<VideoStreamProfiler> profiler_;
  // Guards shutdown/publication state. write() intentionally holds this across
  // publish/replace and captureFrame() so shutdown() cannot tear down the
  // current LiveKit source while a frame handoff is in flight.
  std::mutex mutex_;
  bool is_shutdown_ = false;
  // Sticky across replacement-publish failures so a later successful
  // replacement still reports republished=true.
  bool has_published_ = false;
  // One active LiveKit publication; width/height are meaningful only while
  // both source and track are set.
  std::shared_ptr<livekit::VideoSource> active_source_;
  std::shared_ptr<VideoTrackHandle> active_track_;
  int active_width_ = 0;
  int active_height_ = 0;
};

}  // namespace livekit_ros2_bridge
