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
#include <string>
#include <unordered_map>
#include <vector>

namespace livekit_ros2_bridge
{

namespace video_defaults
{

inline constexpr char kDefaultRosProfileId[] = "default_ros";
inline constexpr char kDefaultRosTransform[] = "";

}  // namespace video_defaults

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

struct ConfiguredVideoSourceConfig
{
  std::string ingress_fragment;
  std::string transform_fragment;
  VideoPublishConfig publish;
};

struct VideoConfig
{
  std::vector<RosTopicRule> ros_topic_rules;
  // Keyed by the trimmed configured-source name used during stream-spec resolution.
  std::unordered_map<std::string, ConfiguredVideoSourceConfig> configured_sources;
  VideoPublishConfig publish;
};

VideoConfig makeDefaultVideoConfig();

}  // namespace livekit_ros2_bridge
