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
#include <variant>
#include <vector>

#include "livekit/room_event_types.h"

namespace livekit_ros2_bridge
{

inline constexpr char kRawImageIngestMode[] = "raw_image";
inline constexpr char kCompressedImageIngestMode[] = "compressed_image";
inline constexpr char kOtherVideoIngestMode[] = "other_video";

// Declared config for one ROS topic rule before it is resolved into a stream spec.
struct RosVideoTopicRule
{
  std::string pattern;
  std::string rule_id;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_config;
};

// Declared config for one other video source before it is resolved into a stream spec.
struct OtherVideoSource
{
  std::string ingress_fragment;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_config;
};

// Declared video configuration. Stream specs resolve from this config, instances own the shared
// live runtime, publishers own one LiveKit publication, and sources produce frames into a sink.
struct VideoStreamConfig
{
  std::vector<RosVideoTopicRule> ros_topic_rules;
  // Keyed by the trimmed other-video-source name used during stream-spec resolution.
  std::unordered_map<std::string, OtherVideoSource> other_video_sources;
  livekit::TrackPublishOptions default_publish_config;
};

inline VideoStreamConfig makeDefaultVideoStreamConfig()
{
  VideoStreamConfig stream_config;
  stream_config.ros_topic_rules.push_back({"/*", "default_ros", "", stream_config.default_publish_config});
  return stream_config;
}

enum class VideoInputKind
{
  RosTopic,
  OtherVideoSource,
};

enum class RosVideoIngestMode
{
  RawImage,
  CompressedImage,
};

struct RosVideoInput
{
  std::string topic;
  std::string interface_type;
  RosVideoIngestMode ingest_mode = RosVideoIngestMode::RawImage;
  std::string rule_id;
  std::string transform_fragment;
};

struct OtherVideoInput
{
  std::string name;
  std::string ingress_fragment;
  std::string transform_fragment;
};

using VideoStreamInput = std::variant<RosVideoInput, OtherVideoInput>;

struct VideoStreamSpec
{
  // Resolved runtime inputs for one shared video stream instance.
  // Stable video runtime key derived from the canonical topic/source identifier:
  // "topic:<normalized topic>" or "other_video:<trimmed other video source name>".
  std::string stream_key;
  // Stable LiveKit track name derived from the same canonical identifier as stream_key.
  // ROS topics keep the legacy slash/colon-to-dot mapping that existing subscribers already use,
  // while other video sources percent-encode their free-form names to keep the suffix reversible.
  // This is client-visible in subscription status and should remain stable for a given stream.
  std::string track_name;

  // Optional operator-facing detail surfaced when the stream is degraded but still addressable.
  std::optional<std::string> degraded_reason;

  // Source-specific data. The active alternative determines which fields are valid.
  VideoStreamInput input;
  // Resolved LiveKit publish config after applying any per-entry overrides to video.publish.*.
  livekit::TrackPublishOptions publish_config;
};

// Returns the ingest mode for supported ROS video interface types and std::nullopt for non-video types.
std::optional<RosVideoIngestMode> classifyRosVideoIngestMode(std::string_view interface_type);
std::string_view rosVideoIngestModeToString(RosVideoIngestMode mode);
std::string_view videoInputKindToString(VideoInputKind kind);
VideoInputKind videoInputKind(const VideoStreamSpec & spec) noexcept;
std::string_view videoIngestModeToString(const VideoStreamSpec & spec);
const RosVideoInput * rosVideoInput(const VideoStreamSpec & spec) noexcept;
RosVideoInput * rosVideoInput(VideoStreamSpec & spec) noexcept;
const OtherVideoInput * otherVideoInput(const VideoStreamSpec & spec) noexcept;
OtherVideoInput * otherVideoInput(VideoStreamSpec & spec) noexcept;
const RosVideoInput & requireRosVideoInput(const VideoStreamSpec & spec);
const OtherVideoInput & requireOtherVideoInput(const VideoStreamSpec & spec);

// Normalizes the topic before matching and identifier generation so equivalent ROS spellings collapse to one runtime.
// The longest rule match wins; same-length matches keep declaration order so config files can express deterministic
// precedence without an extra priority field.
VideoStreamSpec resolveRosVideoTopicSpec(
  const VideoStreamConfig & config, const std::string & topic, const std::string & interface_type);
// Trims the other-video-source name before lookup. The resulting track name percent-encodes that canonical name
// because other-video identifiers are free-form strings, not normalized ROS resource paths.
VideoStreamSpec resolveOtherVideoSourceSpec(const VideoStreamConfig & config, const std::string & requested_name);

}  // namespace livekit_ros2_bridge
