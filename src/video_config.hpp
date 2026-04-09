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
#include <unordered_map>
#include <vector>

namespace livekit_ros2_bridge
{

namespace video_defaults
{

inline constexpr char kDefaultRosProfileId[] = "default_ros";

// clang-format off
inline constexpr char kDefaultImagePipeline[] =
  "rosrawimagesrc ros-topic={topic} ros-reliable=true"
  " ! queue max-size-buffers=2 leaky=downstream"
  " ! videorate drop-only=true ! video/x-raw,framerate=12/1"
  " ! videoconvert"
  " ! vp8enc deadline=1 keyframe-max-dist=16 target-bitrate=900000";

inline constexpr char kDefaultCompressedImagePipeline[] =
  "roscompressedimagesrc ros-topic={topic} ros-reliable=true ! jpegdec"
  " ! queue max-size-buffers=2 leaky=downstream"
  " ! videorate drop-only=true ! video/x-raw,framerate=12/1"
  " ! videoconvert"
  " ! vp8enc deadline=1 keyframe-max-dist=16 target-bitrate=900000";
// clang-format on

}  // namespace video_defaults

/// Pipeline alias → full GStreamer pipeline template string.
/// Alias keys: "image", "compressed_image", "default".
using PipelineMap = std::unordered_map<std::string, std::string>;

enum class VideoSourceKind
{
  RosTopic,
  Pipeline,
};

struct RosTopicRule
{
  std::string pattern;
  std::string id;
  PipelineMap pipelines;
};

struct ConfiguredPipelineSource
{
  // Canonical normalized external name used for lookup and selected_config_key.
  std::string external_name;
  std::string pipeline;
};

struct VideoConfig
{
  std::vector<RosTopicRule> ros_topic_rules;
  // Keyed by normalizeExternalName(...); the stored external_name repeats that canonical key.
  std::unordered_map<std::string, ConfiguredPipelineSource> pipeline_sources;
};

struct SidecarLaunchSpec
{
  // Stable supervisor key: "topic:<normalized topic>" or "external:<normalized external name>".
  std::string sidecar_key;
  // Set only for ROS-topic sources after ROS resource normalization.
  std::string ros_topic;
  // Set only for ROS-topic sources and must pass isSupportedVideoInterfaceType().
  std::string interface_type;
  // Set only for configured pipeline sources after external-name normalization.
  std::string external_name;
  VideoSourceKind source_kind = VideoSourceKind::RosTopic;
  std::string ingest_mode;
  // ROS sources store the matched rule id; pipeline sources store the canonical external name.
  std::string selected_config_key;
  std::optional<std::string> degraded_reason;
  // Tokenized argv appended after `gstreamer-publisher --`.
  std::vector<std::string> source_pipeline;
};

VideoConfig makeDefaultVideoConfig();

// Canonicalizes configured source names so equivalent spellings share one config key and track name.
std::string normalizeExternalName(std::string_view external_name);
std::string videoSourceKindToString(VideoSourceKind kind);

// Resolves against normalized topic patterns. The longest match wins; same-length matches keep declaration order.
// Interface-specific aliases prefer "image"/"compressed_image" and fall back to "default".
SidecarLaunchSpec resolveRosVideoLaunchSpec(
  const VideoConfig & config, const std::string & topic, const std::string & interface_type);
// Normalizes the configured source name before lookup and fills only the pipeline-source fields in the result.
SidecarLaunchSpec resolvePipelineVideoLaunchSpec(const VideoConfig & config, const std::string & external_name);

}  // namespace livekit_ros2_bridge
