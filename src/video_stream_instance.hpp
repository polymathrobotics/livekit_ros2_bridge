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

#include <memory>
#include <mutex>
#include <string>

#include "video_profiling.hpp"
#include "video_stream_runtime.hpp"
#include "video_stream_spec.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;
class RoomConnection;
class VideoTrackPublisher;

// VideoStreamRegistry owns one instance per resolved stream key.
// Each instance owns the input-side VideoFrameSource and its paired LiveKit publisher.
// Those runtime pieces can call back on different threads, so shutdown and shared
// bookkeeping stay local to this object behind mutex_.
class VideoStreamInstance final : public VideoStreamLifecycleObserver
{
public:
  VideoStreamInstance(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * qos_config,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);
  ~VideoStreamInstance();

  VideoStreamInstance(const VideoStreamInstance &) = delete;
  VideoStreamInstance & operator=(const VideoStreamInstance &) = delete;
  VideoStreamInstance(VideoStreamInstance &&) = delete;
  VideoStreamInstance & operator=(VideoStreamInstance &&) = delete;

  // Lazily constructs the input source on first start and reuses it until shutdown().
  // Throws if shutdown() has already begun.
  std::string start();
  // Idempotent. Detaches owned runtime objects under mutex_ and tears them down after
  // unlocking so their shutdown paths never re-enter this instance while the mutex is held.
  void shutdown();

  // Callbacks may arrive from ROS, GStreamer, or LiveKit worker threads, including
  // after shutdown() has started. They only touch local bookkeeping here and never
  // reach back into source_ or publisher_ while teardown is in flight.
  void onTrackPublished(int width, int height, bool republished) override;
  void onTrackUnpublish() override;
  void onSampleUnpackFailed(const std::string & error) override;
  void onCaptureFailed(const std::string & error) override;
  void onPipelineFailed(const std::string & reason) override;
  void onRestartFailed(const std::string & error) override;
  void onPushFailed(const std::string & error) override;

private:
  rclcpp::Node & node_;
  VideoStreamSpec spec_;
  // Borrowed bridge-wide QoS overrides; the owner must outlive this instance.
  const SubscriptionQosConfig * qos_config_;
  std::shared_ptr<VideoStreamProfiler> profiler_;
  // Guards shutdown state and protects owned runtime handles across public methods and callbacks.
  std::mutex mutex_;
  bool is_shutdown_ = false;
  // Tracks whether this instance currently has a published track so duplicate
  // unpublish callbacks stay quiet during teardown.
  bool track_published_ = false;
  // Created on first start() so unused streams do not allocate subscriptions or pipelines.
  std::shared_ptr<VideoFrameSource> source_;
  // Constructed eagerly because every frame source needs a stable sink/publisher reference.
  std::unique_ptr<VideoTrackPublisher> publisher_;
};

}  // namespace livekit_ros2_bridge
