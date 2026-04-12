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

#include <optional>
#include <string>
#include <string_view>

#include "video_stream_config.hpp"

namespace livekit_ros2_bridge
{

inline constexpr char kImageInterfaceType[] = "sensor_msgs/msg/Image";
inline constexpr char kCompressedImageInterfaceType[] = "sensor_msgs/msg/CompressedImage";
inline constexpr char kRawImageIngestMode[] = "raw_image";
inline constexpr char kCompressedImageIngestMode[] = "compressed_image";
inline constexpr char kConfiguredSourceIngestMode[] = "configured_source";

struct RosVideoInterfaceClassification
{
  std::string_view ingest_mode;
};

enum class VideoInputKind
{
  RosTopic,
  ConfiguredSource,
};

struct VideoStreamSpec
{
  // Resolved runtime inputs for one shared video stream instance.
  // Stable video runtime key: "topic:<normalized topic>" or "configured_source:<trimmed configured source name>".
  std::string stream_key;
  // Stable LiveKit track name exposed to subscribers.
  std::string track_name;

  // Set only for ROS-topic sources after ROS resource normalization.
  std::string ros_topic;
  // Set only for ROS-topic sources and must resolve via classifyRosVideoInterfaceType(...).
  std::string interface_type;

  // Set only for configured sources after configured-source-name trimming.
  std::string configured_source_name;

  VideoInputKind input_kind = VideoInputKind::RosTopic;
  std::string ingest_mode;
  // ROS sources store the matched rule_id; configured sources store the canonical trimmed configured source name.
  std::string selected_config_id;
  std::optional<std::string> degraded_reason;

  // Configured sources set this to the configured ingress fragment. ROS sources leave it empty.
  std::string ingress_fragment;
  // Optional GStreamer transform fragment inserted after ingress and before the bridge-owned tail.
  std::string transform_fragment;
  // Resolved LiveKit publish config after applying any per-entry overrides to video.publish.*.
  VideoPublishConfig publish_config;
};

// Returns the ingest contract for supported ROS video types and std::nullopt for non-video types.
std::optional<RosVideoInterfaceClassification> classifyRosVideoInterfaceType(std::string_view interface_type);
// Trims configured source names. Unlike ROS resources, slashes and colons stay significant.
std::string trimConfiguredSourceName(std::string_view configured_source_name);
std::string videoInputKindToString(VideoInputKind kind);

// Resolves against normalized topic patterns. The longest match wins; same-length matches keep declaration order.
VideoStreamSpec resolveRosVideoStreamSpec(
  const VideoStreamConfig & stream_config, const std::string & topic, const std::string & interface_type);
// Trims the configured source name before lookup and fills only the configured-source fields in the result.
VideoStreamSpec resolveConfiguredSourceVideoStreamSpec(
  const VideoStreamConfig & stream_config, const std::string & configured_source_name);

}  // namespace livekit_ros2_bridge
