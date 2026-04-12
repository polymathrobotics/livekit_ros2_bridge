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

// Declared config for one ROS topic rule before it is resolved into a stream spec.
struct RosVideoTopicRule
{
  std::string pattern;
  std::string rule_id;
  std::string transform_fragment;
  VideoPublishConfig publish_config;
};

// Declared config for one configured video source before it is resolved into a stream spec.
struct ConfiguredVideoStreamSource
{
  std::string ingress_fragment;
  std::string transform_fragment;
  VideoPublishConfig publish_config;
};

// Declared video configuration. Stream specs resolve from this config, instances own the shared
// live runtime, publishers own one LiveKit publication, and sources produce frames into a sink.
struct VideoStreamConfig
{
  std::vector<RosVideoTopicRule> ros_topic_rules;
  // Keyed by the trimmed configured-source name used during stream-spec resolution.
  std::unordered_map<std::string, ConfiguredVideoStreamSource> configured_sources;
  VideoPublishConfig default_publish_config;
};

VideoStreamConfig makeDefaultVideoStreamConfig();

}  // namespace livekit_ros2_bridge
