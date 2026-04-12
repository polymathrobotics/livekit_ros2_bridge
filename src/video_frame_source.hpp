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

#include "rclcpp/node.hpp"
#include "subscription_qos.hpp"
#include "video_stream_runtime.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class VideoStreamProfiler;

std::shared_ptr<VideoFrameSource> makeRawRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

std::shared_ptr<VideoFrameSource> makeCompressedRosVideoFrameSource(
  rclcpp::Node & node,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

std::shared_ptr<VideoFrameSource> makeConfiguredSourceVideoFrameSource(
  VideoStreamSpec spec,
  VideoFrameSink & frame_sink,
  VideoStreamLifecycleObserver & lifecycle_observer,
  std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

}  // namespace livekit_ros2_bridge
