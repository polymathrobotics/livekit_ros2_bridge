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
  std::string external_name;
  std::string pipeline;
};

struct VideoConfig
{
  std::vector<RosTopicRule> ros_topic_rules;
  std::unordered_map<std::string, ConfiguredPipelineSource> pipeline_sources;
};

struct SidecarLaunchSpec
{
  std::string sidecar_key;
  std::string ros_topic;
  std::string interface_type;
  std::string external_name;
  VideoSourceKind source_kind = VideoSourceKind::RosTopic;
  std::string ingest_mode;
  std::string selected_config_key;
  std::optional<std::string> degraded_reason;
  std::vector<std::string> source_pipeline;
};

VideoConfig makeDefaultVideoConfig();

std::string normalizeExternalName(std::string_view external_name);
std::string videoSourceKindToString(VideoSourceKind kind);

SidecarLaunchSpec resolveRosVideoLaunchSpec(
  const VideoConfig & config, const std::string & topic, const std::string & interface_type);
SidecarLaunchSpec resolvePipelineVideoLaunchSpec(const VideoConfig & config, const std::string & external_name);

}  // namespace livekit_ros2_bridge
