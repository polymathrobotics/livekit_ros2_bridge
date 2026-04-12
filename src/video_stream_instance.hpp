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

// VideoStreamRegistry owns one VideoStreamInstance per resolved stream key.
// Each instance owns one live runtime: the input-side VideoFrameSource plus the paired
// VideoTrackPublisher for the matching LiveKit video publication.
class VideoStreamInstance final : public VideoStreamLifecycleObserver
{
public:
  VideoStreamInstance(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config);
  ~VideoStreamInstance();

  VideoStreamInstance(const VideoStreamInstance &) = delete;
  VideoStreamInstance & operator=(const VideoStreamInstance &) = delete;
  VideoStreamInstance(VideoStreamInstance &&) = delete;
  VideoStreamInstance & operator=(VideoStreamInstance &&) = delete;

  void onVideoTrackPublished(int width, int height, bool republished) override;
  void onVideoTrackUnpublishing() override;
  void onVideoStreamSampleUnpackFailed(const std::string & error) override;
  void onVideoStreamCaptureFailed(const std::string & error) override;
  void onVideoStreamPipelineFailed(const std::string & reason) override;
  void onVideoStreamRestartFailed(const std::string & error) override;
  void onVideoStreamPushFailed(const std::string & error) override;

  std::string ensureRunning();
  void shutdown();

private:
  void logRuntimeError(const char * event_name, const std::string & error);

  std::shared_ptr<VideoFrameSource> createFrameSourceLocked();

  rclcpp::Node & node_;
  VideoStreamSpec spec_;
  const SubscriptionQosConfig * subscription_qos_config_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  bool has_published_track_ = false;
  int published_width_ = 0;
  int published_height_ = 0;
  std::string last_runtime_error_;
  std::shared_ptr<VideoFrameSource> frame_source_;
  std::unique_ptr<VideoTrackPublisher> video_track_publisher_;
};

}  // namespace livekit_ros2_bridge
