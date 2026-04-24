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
#include <string>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "room_connection.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;

class GStreamerVideoStream;
class RosVideoStream;

// Owns one concrete video ingress runtime and one lazily republished LiveKit
// video track on egress.
class VideoTrackPublisher final
{
public:
  static std::shared_ptr<VideoTrackPublisher> create(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
    RoomConnection & room_connection,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * qos_config);

  // Test-only: construct a publisher without a frame source so the publish/unpublish
  // flow can be exercised via direct captureFrame() calls.
  VideoTrackPublisher(RoomConnection & room_connection, VideoStreamSpec spec);

  ~VideoTrackPublisher();

  VideoTrackPublisher(const VideoTrackPublisher &) = delete;
  VideoTrackPublisher & operator=(const VideoTrackPublisher &) = delete;
  VideoTrackPublisher(VideoTrackPublisher &&) = delete;
  VideoTrackPublisher & operator=(VideoTrackPublisher &&) = delete;

  const VideoStreamSpec & spec() const
  {
    return spec_;
  }

  void captureFrame(const livekit::VideoFrame & frame, std::int64_t timestamp_us);

private:
  friend class GStreamerVideoStream;
  friend class RosVideoStream;

  // Streams may fire these from ROS, GStreamer, or LiveKit worker threads,
  // including after close() has started.
  void onSampleUnpackFailed(const std::string & error);
  void onCaptureFailed(const std::string & error);
  void onRestartFailed(const std::string & error);
  void onPushFailed(const std::string & error);

  void close();

  RoomConnection & room_connection_;
  VideoStreamSpec spec_;

  std::mutex mutex_;
  bool is_closed_ = false;
  bool was_published_ = false;
  std::shared_ptr<RosVideoStream> ros_stream_;
  std::unique_ptr<GStreamerVideoStream> gstreamer_stream_;
  std::shared_ptr<livekit::VideoSource> video_source_;
  std::shared_ptr<livekit::LocalVideoTrack> published_video_track_;
};

}  // namespace livekit_ros2_bridge
