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
#include <vector>

#include "rclcpp/node.hpp"
#include "subscription_qos.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class VideoFrameSink
{
public:
  virtual ~VideoFrameSink() = default;

  virtual void handleFrame(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us) = 0;
};

// VideoStreamInstance owns one frame source on the input side and wires it to a
// VideoTrackPublisher through this sink interface.
class VideoFrameSource
{
public:
  virtual ~VideoFrameSource() = default;

  virtual void ensureRunning() = 0;
  virtual void shutdown() = 0;
};

std::shared_ptr<VideoFrameSource> makeRawRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink);

std::shared_ptr<VideoFrameSource> makeCompressedRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink);

std::shared_ptr<VideoFrameSource> makeConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec, VideoFrameSink & frame_sink);

}  // namespace livekit_ros2_bridge
