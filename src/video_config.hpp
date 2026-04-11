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
#include <string_view>
#include <unordered_map>
#include <vector>

namespace livekit_ros2_bridge
{

inline constexpr char kImageInterfaceType[] = "sensor_msgs/msg/Image";
inline constexpr char kCompressedImageInterfaceType[] = "sensor_msgs/msg/CompressedImage";
inline constexpr char kRawImageIngestMode[] = "raw_image";
inline constexpr char kCompressedImageIngestMode[] = "compressed_image";
inline constexpr char kConfiguredSourceIngestMode[] = "configured_source";

namespace video_defaults
{

inline constexpr char kDefaultRosProfileId[] = "default_ros";
inline constexpr char kDefaultRosTransform[] = "";

}  // namespace video_defaults

struct RosVideoSourceClassification
{
  std::string_view ingest_mode;
};

enum class VideoSourceKind
{
  RosTopic,
  ConfiguredSource,
};

enum class VideoPublishCodec
{
  Auto,
  Vp8,
  H264,
  Av1,
  Vp9,
  H265,
};

enum class VideoPublishSimulcast
{
  Auto,
  Enabled,
  Disabled,
};

struct VideoPublishConfig
{
  VideoPublishCodec codec = VideoPublishCodec::Auto;
  std::uint64_t max_bitrate_bps = 0;
  double max_framerate = 0.0;
  VideoPublishSimulcast simulcast = VideoPublishSimulcast::Auto;
};

struct RosTopicRule
{
  std::string pattern;
  std::string id;
  std::string transform;
  VideoPublishConfig publish;
};

struct ConfiguredSource
{
  std::string source;
  std::string transform;
  VideoPublishConfig publish;
};

struct VideoConfig
{
  std::vector<RosTopicRule> ros_topic_rules;
  // Keyed by normalizeConfiguredSourceName(...).
  std::unordered_map<std::string, ConfiguredSource> configured_sources;
  VideoPublishConfig publish;
};

struct VideoStreamSpec
{
  // Stable video runtime key: "topic:<normalized topic>" or "configured_source:<normalized configured source name>".
  std::string stream_key;
  // Stable LiveKit track name exposed to subscribers.
  std::string track_name;
  // Set only for ROS-topic sources after ROS resource normalization.
  std::string ros_topic;
  // Set only for ROS-topic sources and must resolve via classifyRosVideoInterfaceType(...).
  std::string interface_type;
  // Set only for configured sources after configured-source-name normalization.
  std::string configured_source_name;
  VideoSourceKind source_kind = VideoSourceKind::RosTopic;
  std::string ingest_mode;
  // ROS sources store the matched rule id; configured sources store the canonical configured source name.
  std::string selected_config_key;
  std::optional<std::string> degraded_reason;
  // Configured sources set this to the configured source fragment. ROS sources leave it empty.
  std::string source_description;
  // Optional GStreamer transform fragment inserted after ingress and before the bridge tail.
  std::string transform_description;
  // Resolved LiveKit publish config after applying any per-entry overrides to video.publish.*.
  VideoPublishConfig publish_config;
};

VideoConfig makeDefaultVideoConfig();

// Returns the ingest contract for supported ROS video types and std::nullopt for non-video types.
std::optional<RosVideoSourceClassification> classifyRosVideoInterfaceType(std::string_view interface_type);
// Canonicalizes configured source names so equivalent spellings share one config key and track name.
std::string normalizeConfiguredSourceName(std::string_view configured_source_name);
std::string videoSourceKindToString(VideoSourceKind kind);

// Resolves against normalized topic patterns. The longest match wins; same-length matches keep declaration order.
VideoStreamSpec resolveRosVideoStreamSpec(
  const VideoConfig & config, const std::string & topic, const std::string & interface_type);
// Normalizes the configured source name before lookup and fills only the configured-source fields in the result.
VideoStreamSpec resolveConfiguredSourceVideoStreamSpec(
  const VideoConfig & config, const std::string & configured_source_name);

}  // namespace livekit_ros2_bridge
