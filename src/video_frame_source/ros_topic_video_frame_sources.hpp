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
#include <optional>
#include <string>

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_qos.hpp"
#include "video_frame_source/video_pipeline_frame_source.hpp"

#include <gst/video/video-format.h>

namespace livekit_ros2_bridge
{

struct FrameLayout
{
  // This reflects the observed raw frame layout used for appsrc caps and video
  // metadata. Any change requires rebuilding the pipeline instead of mutating
  // the existing appsrc in place.
  int width = 0;
  int height = 0;
  GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
  std::uint32_t stride = 0;
};

enum class CompressedImageCodec
{
  kJpeg,
  kPng,
};

class RawRosVideoFrameSource final : public VideoPipelineFrameSource
{
public:
  RawRosVideoFrameSource(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * qos_config,
    VideoFrameSink & sink,
    VideoStreamLifecycleObserver & observer,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  void start() override;
  void shutdown() override;

private:
  rclcpp::Node & node_;
  // Non-owning bridge-wide QoS policy. The pointed-to config must outlive this source.
  const SubscriptionQosConfig * qos_config_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::optional<FrameLayout> layout_;

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & image);

  void startLocked(const FrameLayout & layout);
  void pushLocked(const sensor_msgs::msg::Image & image);

  void resetLocked() override;
};

class CompressedRosVideoFrameSource final : public VideoPipelineFrameSource
{
public:
  CompressedRosVideoFrameSource(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * qos_config,
    VideoFrameSink & sink,
    VideoStreamLifecycleObserver & observer,
    std::shared_ptr<VideoStreamProfiler> profiler = nullptr);

  void start() override;
  void shutdown() override;

private:
  rclcpp::Node & node_;
  // Non-owning bridge-wide QoS policy. The pointed-to config must outlive this source.
  const SubscriptionQosConfig * qos_config_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  std::optional<CompressedImageCodec> codec_;

  void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image);

  void startLocked(CompressedImageCodec codec);
  void pushLocked(const sensor_msgs::msg::CompressedImage & image);

  void resetLocked() override;
};

}  // namespace livekit_ros2_bridge
