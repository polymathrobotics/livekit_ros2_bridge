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
#include "ros_node_interfaces.hpp"
#include "video_profiling.hpp"
#include "video_stream_runtime.hpp"
#include "video_stream_spec.hpp"

namespace livekit
{
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;

// Owns one video subscription runtime: the paired VideoFrameSource on ingress and
// one lazily republished LiveKit video track on egress.
class VideoTrackPublisher final : public VideoFrameSink, private VideoStreamLifecycleObserver
{
public:
  static std::shared_ptr<VideoTrackPublisher> create(
    SubscriptionNodeInterfaces interfaces,
    RoomConnection & room_connection,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * qos_config,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  // Test-only: construct a publisher without a frame source so the publish/unpublish
  // flow can be exercised via direct write() calls.
  VideoTrackPublisher(
    RoomConnection & room_connection, VideoStreamSpec spec, std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  ~VideoTrackPublisher();

  VideoTrackPublisher(const VideoTrackPublisher &) = delete;
  VideoTrackPublisher & operator=(const VideoTrackPublisher &) = delete;
  VideoTrackPublisher(VideoTrackPublisher &&) = delete;
  VideoTrackPublisher & operator=(VideoTrackPublisher &&) = delete;

  const VideoStreamSpec & spec() const
  {
    return spec_;
  }

  void write(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us) override;

private:
  class Publication;

  // Frame sources may fire these from ROS, GStreamer, or LiveKit worker threads,
  // including after close() has started.
  void onSampleUnpackFailed(const std::string & error) override;
  void onCaptureFailed(const std::string & error) override;
  void onPipelineFailed(const std::string & reason) override;
  void onRestartFailed(const std::string & error) override;
  void onPushFailed(const std::string & error) override;

  void close();

  RoomConnection & room_connection_;
  VideoStreamSpec spec_;
  std::shared_ptr<VideoStreamProfiler> profiler_;

  std::mutex mutex_;
  bool is_closed_ = false;
  bool was_published_ = false;
  std::shared_ptr<VideoFrameSource> frame_source_;
  std::unique_ptr<Publication> publication_;
};

}  // namespace livekit_ros2_bridge
