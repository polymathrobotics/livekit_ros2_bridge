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

#include "video_stream_spec.hpp"

#include <stdexcept>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kTopicKeyPrefix[] = "topic";
constexpr char kConfiguredSourceKeyPrefix[] = "other_video";
constexpr char kTopicTrackPrefix[] = "ros.video.";
constexpr char kConfiguredSourceTrackPrefix[] = "ros.video.other.";
constexpr char kHexDigits[] = "0123456789ABCDEF";
constexpr char kUnnamedTrackSuffix[] = "unnamed";
const auto kLogger = rclcpp::get_logger("video_stream_spec");

// ROS-topic track names intentionally keep the historical slash/colon-to-dot mapping
// that existing subscribers already consume, rather than percent-encoding the topic.
// This compatibility mapping is not fully reversible; stream_key remains the authoritative
// runtime identity when the bridge needs an exact normalized ROS topic.
std::string makeTopicTrackSuffix(std::string_view topic)
{
  std::string suffix;
  suffix.reserve(topic.size());
  for (char ch : topic) {
    if (ch != '/' && ch != ':') {
      suffix.push_back(ch);
      continue;
    }
    if (!suffix.empty() && suffix.back() != '.') {
      suffix.push_back('.');
    }
  }

  const auto first_non_dot = suffix.find_first_not_of('.');
  if (first_non_dot == std::string::npos) {
    return {};
  }
  suffix.erase(0, first_non_dot);
  suffix.erase(suffix.find_last_not_of('.') + 1U);
  return suffix;
}

std::string makeStreamKey(std::string_view prefix, std::string_view canonical_name)
{
  std::string stream_key;
  stream_key.reserve(prefix.size() + 1U + canonical_name.size());
  stream_key.append(prefix);
  stream_key.push_back(':');
  stream_key.append(canonical_name);
  return stream_key;
}

std::string makeTrackName(std::string_view prefix, std::string_view suffix)
{
  std::string track_name;
  track_name.reserve(prefix.size() + suffix.size());
  track_name.append(prefix);
  track_name.append(suffix);
  return track_name;
}

std::string makeTopicTrackName(std::string_view normalized_topic)
{
  const std::string suffix = makeTopicTrackSuffix(normalized_topic);
  if (suffix.empty()) {
    return makeTrackName(kTopicTrackPrefix, kUnnamedTrackSuffix);
  }
  return makeTrackName(kTopicTrackPrefix, suffix);
}

bool isUnreservedTrackByte(unsigned char byte)
{
  return (byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z') || (byte >= '0' && byte <= '9') || byte == '-' ||
         byte == '.' || byte == '_' || byte == '~';
}

// Other-video names are free-form identifiers, so percent-encode reserved bytes
// to keep the client-visible track suffix reversible and avoid dot-mapping collisions.
std::string encodeConfiguredSourceTrackSuffix(std::string_view name)
{
  std::string suffix;
  suffix.reserve(name.size() * 3U);
  for (const char ch : name) {
    const auto byte = static_cast<unsigned char>(ch);
    if (isUnreservedTrackByte(byte)) {
      suffix.push_back(static_cast<char>(byte));
      continue;
    }
    suffix.push_back('%');
    suffix.push_back(kHexDigits[byte >> 4U]);
    suffix.push_back(kHexDigits[byte & 0x0FU]);
  }
  return suffix;
}

std::string makeConfiguredSourceTrackName(std::string_view name)
{
  return makeTrackName(kConfiguredSourceTrackPrefix, encodeConfiguredSourceTrackSuffix(name));
}

const RosVideoTopicRule & selectBestMatchingRosVideoTopicRule(
  const std::vector<RosVideoTopicRule> & rules, std::string_view normalized_topic)
{
  const RosVideoTopicRule * match = nullptr;
  for (const auto & rule : rules) {
    if (!rosResourceMatchesPattern(normalized_topic, rule.pattern)) {
      continue;
    }
    // Prefer the longest matching pattern; same-length matches keep declaration order.
    const auto pattern_size = rule.pattern.size();
    if (match != nullptr && pattern_size <= match->pattern.size()) {
      continue;
    }
    match = &rule;
  }
  if (match == nullptr) {
    LogEvent(kLogger, "video_stream_spec_rejected")
      .field("resource", normalized_topic)
      .field("reason", "no_matching_ros_topic_rule")
      .field("configured_rules", rules.size())
      .warn();
    throw std::runtime_error("no matching video rule for topic '" + std::string(normalized_topic) + "'");
  }
  return *match;
}

}  // namespace

std::optional<std::string_view> classifyRosVideoIngestMode(std::string_view interface_type)
{
  if (interface_type == kImageInterfaceType) {
    return kRawImageIngestMode;
  }
  if (interface_type == kCompressedImageInterfaceType) {
    return kCompressedImageIngestMode;
  }
  return std::nullopt;
}

std::string videoInputKindToString(VideoInputKind kind)
{
  if (kind == VideoInputKind::RosTopic) {
    return "ros_topic";
  }
  return "other_video";
}

VideoStreamSpec resolveRosVideoTopicSpec(
  const VideoStreamConfig & config, const std::string & topic, const std::string & interface_type)
{
  const std::string normalized_topic = normalizeRosResourceName(topic);
  if (normalized_topic.empty()) {
    throw std::invalid_argument("Invalid ROS topic.");
  }
  const auto ingest_mode = classifyRosVideoIngestMode(interface_type);
  if (!ingest_mode.has_value()) {
    LogEvent(kLogger, "video_stream_spec_rejected")
      .field("resource", normalized_topic)
      .field("interface_type", interface_type)
      .field("reason", "unsupported_ros_interface_type")
      .warn();
    throw std::invalid_argument("ROS topic is not a supported video type.");
  }

  const auto & rule = selectBestMatchingRosVideoTopicRule(config.ros_topic_rules, normalized_topic);

  VideoStreamSpec spec;
  spec.stream_key = makeStreamKey(kTopicKeyPrefix, normalized_topic);
  spec.track_name = makeTopicTrackName(normalized_topic);
  spec.input_kind = VideoInputKind::RosTopic;
  spec.ros_topic = normalized_topic;
  spec.interface_type = interface_type;
  spec.ingest_mode = std::string(*ingest_mode);

  spec.config_id = rule.rule_id;
  spec.transform_fragment = rule.transform_fragment;
  spec.publish_config = rule.publish_config;
  return spec;
}

VideoStreamSpec resolveConfiguredVideoSourceSpec(const VideoStreamConfig & config, const std::string & source_name)
{
  const std::string name = trim(source_name);
  if (name.empty()) {
    throw std::invalid_argument("Invalid other video name.");
  }

  const auto it = config.configured_sources.find(name);
  if (it == config.configured_sources.end()) {
    throw std::invalid_argument("Unknown other video source '" + name + "'.");
  }

  const auto & source_config = it->second;

  VideoStreamSpec spec;
  spec.stream_key = makeStreamKey(kConfiguredSourceKeyPrefix, name);
  spec.track_name = makeConfiguredSourceTrackName(name);
  spec.input_kind = VideoInputKind::ConfiguredSource;
  spec.source_name = name;
  spec.config_id = name;

  spec.ingress_fragment = source_config.ingress_fragment;
  spec.transform_fragment = source_config.transform_fragment;
  spec.ingest_mode = kConfiguredSourceIngestMode;
  spec.publish_config = source_config.publish_config;

  return spec;
}

}  // namespace livekit_ros2_bridge
