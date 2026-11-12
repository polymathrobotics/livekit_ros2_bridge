// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <gst/video/video.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "core/subscription_qos.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/pipeline_failure_handler.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/stream_spec.hpp"

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
  // A failure stops the current pipeline; the next accepted ROS frame rebuilds
  // appsrc state.
  void onPipelineFailure(const std::string & reason);
  void stopPipelineAfterFailure();

  void onRawImage(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image);

  void startRawPipelineLocked(const RawLayout & layout);
  void startCompressedPipelineLocked(const std::string & codec);
  void startPipelineLocked(const std::string & appsrc);
  bool currentPipelineFailedLocked() const;
  void pushRawLocked(const sensor_msgs::msg::Image & image);
  // Stops the pipeline and forgets its appsrc caps. It stops first, so a failure the
  // old pipeline reports until then is cancelled instead of acting on the next one.
  void stopPipelineLocked();

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
  // Serialises ROS callbacks, failure handling and close(); the GStreamer callbacks
  // never take it and touch only the atomics below.
  std::mutex mutex_;
  std::atomic<bool> is_shutdown_{false};
  // Each pipeline start bumps the generation under mutex_; a failure records the generation
  // current when it is reported, so handling that runs after a replacement can tell it is stale.
  // The generation starts above failed_generation_, so no pipeline counts as failed before one starts.
  std::atomic<std::uint64_t> pipeline_generation_{1};
  std::atomic<std::uint64_t> failed_generation_{0};
  utils::PipelineFailureHandler failure_handler_;
};

}  // namespace livekit_ros2_bridge::video
