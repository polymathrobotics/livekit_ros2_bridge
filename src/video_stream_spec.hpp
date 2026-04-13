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

enum class VideoInputKind
{
  RosTopic,
  ConfiguredSource,
};

struct VideoStreamSpec
{
  // Resolved runtime inputs for one shared video stream instance.
  // Stable video runtime key derived from the canonical topic/source identifier:
  // "topic:<normalized topic>" or "configured_source:<trimmed configured source name>".
  std::string stream_key;
  // Stable LiveKit track name derived from the same canonical identifier as stream_key.
  // ROS topics keep the legacy slash/colon-to-dot mapping that existing subscribers already use,
  // while configured sources percent-encode their free-form names to keep the suffix reversible.
  // This is client-visible in subscription status and should remain stable for a given stream.
  std::string track_name;

  // Set only for ROS-topic sources after ROS resource normalization.
  std::string ros_topic;
  // Set only for ROS-topic sources and must resolve via classifyRosVideoIngestMode(...).
  std::string interface_type;

  // Set only for configured sources after configured-source-name trimming.
  std::string source_name;

  VideoInputKind input_kind = VideoInputKind::RosTopic;
  std::string ingest_mode;
  // ROS sources store the matched rule_id; configured sources store the canonical trimmed configured source name.
  std::string config_id;
  // Optional operator-facing detail surfaced when the stream is degraded but still addressable.
  std::optional<std::string> degraded_reason;

  // Configured sources set this to the configured ingress fragment. ROS sources leave it empty.
  std::string ingress_fragment;
  // Optional GStreamer transform fragment inserted after ingress and before the bridge-owned tail.
  std::string transform_fragment;
  // Resolved LiveKit publish config after applying any per-entry overrides to video.publish.*.
  VideoPublishConfig publish_config;
};

// Returns the ingest mode for supported ROS video interface types and std::nullopt for non-video types.
std::optional<std::string_view> classifyRosVideoIngestMode(std::string_view interface_type);
std::string videoInputKindToString(VideoInputKind kind);

// Normalizes the topic before matching and identifier generation so equivalent ROS spellings collapse to one runtime.
// The longest rule match wins; same-length matches keep declaration order so config files can express deterministic
// precedence without an extra priority field.
VideoStreamSpec resolveRosVideoTopicSpec(
  const VideoStreamConfig & config, const std::string & topic, const std::string & interface_type);
// Trims the configured source name before lookup. The resulting track name percent-encodes that canonical name
// because configured-source identifiers are free-form strings, not normalized ROS resource paths.
VideoStreamSpec resolveConfiguredVideoSourceSpec(const VideoStreamConfig & config, const std::string & source_name);

}  // namespace livekit_ros2_bridge
