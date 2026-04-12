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

struct RawSourceConfig
{
  int width = 0;
  int height = 0;
  GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
  std::uint32_t stride = 0;
};

bool operator==(const RawSourceConfig & lhs, const RawSourceConfig & rhs);
bool operator!=(const RawSourceConfig & lhs, const RawSourceConfig & rhs);

class RawRosVideoFrameSource final : public VideoPipelineFrameSource
{
public:
  RawRosVideoFrameSource(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config,
    VideoFrameSink & frame_sink,
    VideoStreamLifecycleObserver & lifecycle_observer);

  void ensureRunning() override;
  void shutdown() override;

private:
  void createRosSubscriptionLocked();
  void handleRawImageMessage(const sensor_msgs::msg::Image::ConstSharedPtr & message);
  void startRawRosPipelineLocked(const RawSourceConfig & config);
  void pushRawImageLocked(const sensor_msgs::msg::Image & message, const RawSourceConfig & config);
  void resetSourceStateLocked() override;

  rclcpp::Node & node_;
  const SubscriptionQosConfig * subscription_qos_config_;
  bool first_input_logged_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::optional<RawSourceConfig> raw_source_config_;
};

class CompressedRosVideoFrameSource final : public VideoPipelineFrameSource
{
public:
  CompressedRosVideoFrameSource(
    rclcpp::Node & node,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config,
    VideoFrameSink & frame_sink,
    VideoStreamLifecycleObserver & lifecycle_observer);

  void ensureRunning() override;
  void shutdown() override;

private:
  void createRosSubscriptionLocked();
  void handleCompressedImageMessage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message);
  void startCompressedRosPipelineLocked(const std::string & format);
  void pushCompressedImageLocked(const sensor_msgs::msg::CompressedImage & message);
  void resetSourceStateLocked() override;

  rclcpp::Node & node_;
  const SubscriptionQosConfig * subscription_qos_config_;
  bool first_input_logged_ = false;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  std::string compressed_format_;
};

}  // namespace livekit_ros2_bridge
