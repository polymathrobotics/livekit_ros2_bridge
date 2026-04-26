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
#include <string>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_qos.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/pipeline_failure_handler.hpp"
#include "video/stream_spec.hpp"

#include <gst/video/video-format.h>

namespace livekit_ros2_bridge::video
{

class TrackPublisher;

struct RawLayout
{
  int width = 0;
  int height = 0;
  GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
  std::uint32_t stride = 0;
};

class RosStream final : public std::enable_shared_from_this<RosStream>
{
public:
  // The publisher must outlive this stream; `qos_config` is borrowed and may be null.
  RosStream(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
    StreamSpec spec,
    const SubscriptionQosConfig * qos_config,
    TrackPublisher & publisher);
  ~RosStream();

  RosStream(const RosStream &) = delete;
  RosStream & operator=(const RosStream &) = delete;
  RosStream(RosStream &&) = delete;
  RosStream & operator=(RosStream &&) = delete;

  // Requires shared_ptr ownership because subscription callbacks capture weak ownership.
  void start();
  void close();

private:
  bool isShutdown() const;
  // A failure stops the current pipeline; the next accepted ROS frame rebuilds
  // appsrc state.
  void onPipelineFailure(const std::string & reason);
  void stopPipelineAfterFailure();

  void onRawImage(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image);

  void startRawPipelineLocked(const RawLayout & layout);
  void startCompressedPipelineLocked(const std::string & codec);
  void pushRawLocked(const sensor_msgs::msg::Image & image);
  void resetPipelineStateLocked();

  StreamSpec spec_;
  TrackPublisher & publisher_;
  GStreamerPipeline pipeline_;
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface>
    node_interfaces_;
  const SubscriptionQosConfig * qos_config_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  // Current appsrc caps; raw layout or codec changes restart the pipeline on the
  // next accepted frame.
  std::optional<RawLayout> raw_layout_;
  std::optional<std::string> codec_;
  // Protects state shared by ROS callbacks, GStreamer callbacks, and close().
  mutable std::mutex mutex_;
  bool is_shutdown_ = false;
  PipelineFailureHandler failure_handler_;
};

}  // namespace livekit_ros2_bridge::video
