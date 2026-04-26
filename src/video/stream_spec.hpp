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
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge::video
{

struct RosTopicRule
{
  RosResourcePattern pattern;
  std::string rule_id;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_options;
};

struct OtherSource
{
  std::string source_fragment;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_options;
};

struct StreamConfig
{
  std::vector<RosTopicRule> ros_topic_rules;
  // Keyed by the trimmed configured source name.
  std::unordered_map<std::string, OtherSource> other_sources;
  livekit::TrackPublishOptions default_publish_options;
};

inline StreamConfig makeDefaultConfig()
{
  StreamConfig config;
  config.ros_topic_rules.push_back(
    {RosResourcePattern::rootSubtree(), "default_ros", "", config.default_publish_options});
  return config;
}

enum class RosIngestMode
{
  RawImage,
  CompressedImage,
};

struct RosInput
{
  std::string topic;
  std::string interface_type;
  RosIngestMode ingest_mode = RosIngestMode::RawImage;
  std::string rule_id;
  std::string transform_fragment;
};

struct OtherInput
{
  std::string name;
  std::string source_fragment;
  std::string transform_fragment;
};

using StreamInput = std::variant<RosInput, OtherInput>;

struct StreamSpec
{
  // Stable runtime key: "topic:<normalized topic>" or "other_video:<trimmed source name>".
  std::string stream_key;
  // LiveKit track name: legacy lossy ROS suffixes, reversible other-video suffixes.
  std::string track_name;

  StreamInput input;
  livekit::TrackPublishOptions publish_options;
};

std::optional<RosIngestMode> classifyRosIngestMode(std::string_view interface_type);

const RosInput & requireRosInput(const StreamSpec & spec);
const OtherInput & requireOtherInput(const StreamSpec & spec);

// ROS topics are normalized before matching and identifier generation. Longest match wins; ties keep declaration order.
StreamSpec resolveRosTopicSpec(
  const StreamConfig & config, const std::string & requested_topic, const std::string & interface_type);
StreamSpec resolveOtherSourceSpec(const StreamConfig & config, const std::string & source_name);

}  // namespace livekit_ros2_bridge::video
