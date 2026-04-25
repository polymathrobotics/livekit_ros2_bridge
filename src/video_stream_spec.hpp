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

struct RosVideoTopicRule
{
  std::string pattern;
  std::string rule_id;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_options;
};

struct OtherVideoSource
{
  std::string ingress_fragment;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_options;
};

struct VideoStreamConfig
{
  std::vector<RosVideoTopicRule> ros_topic_rules;
  // Keyed by the trimmed other-video-source name.
  std::unordered_map<std::string, OtherVideoSource> other_video_sources;
  livekit::TrackPublishOptions default_publish_options;
};

inline VideoStreamConfig makeDefaultVideoStreamConfig()
{
  VideoStreamConfig config;
  config.ros_topic_rules.push_back({"/*", "default_ros", "", config.default_publish_options});
  return config;
}

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
  // Stable runtime key: "topic:<normalized topic>" or "other_video:<trimmed source name>".
  std::string stream_key;
  // LiveKit track name: legacy lossy ROS suffixes, reversible other-video suffixes.
  std::string track_name;

  VideoStreamInput input;
  livekit::TrackPublishOptions publish_options;
};

std::optional<RosVideoIngestMode> classifyRosVideoIngestMode(std::string_view interface_type);

const RosVideoInput & requireRosVideoInput(const VideoStreamSpec & spec);
const OtherVideoInput & requireOtherVideoInput(const VideoStreamSpec & spec);

// ROS topics are normalized before matching and identifier generation. Longest match wins; ties keep declaration order.
VideoStreamSpec resolveRosVideoTopicSpec(
  const VideoStreamConfig & config, const std::string & requested_topic, const std::string & interface_type);
VideoStreamSpec resolveOtherVideoSourceSpec(const VideoStreamConfig & config, const std::string & source_name);

}  // namespace livekit_ros2_bridge
