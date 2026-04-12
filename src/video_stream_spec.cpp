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

#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kTopicStreamKeyPrefix[] = "topic";
constexpr char kConfiguredSourceStreamKeyPrefix[] = "configured_source";
constexpr char kTopicTrackNamePrefix[] = "ros.video.";
constexpr char kConfiguredSourceTrackNamePrefix[] = "ros.video.configured_source.";
constexpr char kPercentEncodingHexDigits[] = "0123456789ABCDEF";

std::string makeVideoStreamKey(std::string_view prefix, const std::string & resource)
{
  return std::string(prefix) + ":" + resource;
}

std::string makeRosTrackSuffix(const std::string & resource)
{
  std::string suffix;
  suffix.reserve(resource.size());
  for (char ch : resource) {
    if (ch == '/' || ch == ':') {
      if (!suffix.empty() && suffix.back() != '.') {
        suffix.push_back('.');
      }
      continue;
    }
    suffix.push_back(ch);
  }

  while (!suffix.empty() && suffix.front() == '.') {
    suffix.erase(suffix.begin());
  }
  while (!suffix.empty() && suffix.back() == '.') {
    suffix.pop_back();
  }
  return suffix;
}

bool isRfc3986Unreserved(unsigned char byte)
{
  return (byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z') || (byte >= '0' && byte <= '9') || byte == '-' ||
         byte == '.' || byte == '_' || byte == '~';
}

std::string percentEncodeTrackSuffix(std::string_view resource)
{
  std::string suffix;
  suffix.reserve(resource.size() * 3U);
  for (const char ch : resource) {
    const auto byte = static_cast<unsigned char>(ch);
    if (isRfc3986Unreserved(byte)) {
      suffix.push_back(static_cast<char>(byte));
      continue;
    }
    suffix.push_back('%');
    suffix.push_back(kPercentEncodingHexDigits[byte >> 4U]);
    suffix.push_back(kPercentEncodingHexDigits[byte & 0x0FU]);
  }
  return suffix;
}

std::string makeVideoTrackName(std::string_view prefix, const std::string & resource)
{
  const std::string suffix = makeRosTrackSuffix(resource);
  return suffix.empty() ? std::string(prefix) + "unnamed" : std::string(prefix) + suffix;
}

std::string makeConfiguredSourceTrackName(std::string_view prefix, const std::string & configured_source_name)
{
  const std::string suffix = percentEncodeTrackSuffix(configured_source_name);
  return suffix.empty() ? std::string(prefix) + "unnamed" : std::string(prefix) + suffix;
}

}  // namespace

std::optional<RosVideoInterfaceClassification> classifyRosVideoInterfaceType(std::string_view interface_type)
{
  if (interface_type == kImageInterfaceType) {
    return RosVideoInterfaceClassification{kRawImageIngestMode};
  }
  if (interface_type == kCompressedImageInterfaceType) {
    return RosVideoInterfaceClassification{kCompressedImageIngestMode};
  }
  return std::nullopt;
}

std::string trimConfiguredSourceName(std::string_view configured_source_name)
{
  return trim(configured_source_name);
}

std::string videoInputKindToString(VideoInputKind kind)
{
  if (kind == VideoInputKind::RosTopic) {
    return "ros_topic";
  }
  return "configured_source";
}

VideoStreamSpec resolveRosVideoStreamSpec(
  const VideoStreamConfig & stream_config, const std::string & topic, const std::string & interface_type)
{
  const std::string normalized = normalizeRosResourceName(topic);
  if (normalized.empty()) {
    throw std::invalid_argument("Invalid ROS topic.");
  }
  const auto interface_classification = classifyRosVideoInterfaceType(interface_type);
  if (!interface_classification.has_value()) {
    throw std::invalid_argument("ROS topic is not a supported video type.");
  }

  const RosVideoTopicRule * matched_rule = nullptr;
  std::size_t best_len = 0;
  // Update only on a strictly longer pattern so same-length matches stay first-declared.
  for (const auto & rule : stream_config.ros_topic_rules) {
    const bool is_better_match = matched_rule == nullptr || rule.pattern.size() > best_len;
    if (rosResourceMatchesPattern(normalized, rule.pattern) && is_better_match) {
      matched_rule = &rule;
      best_len = rule.pattern.size();
    }
  }
  if (matched_rule == nullptr) {
    throw std::runtime_error("no matching video rule for topic '" + normalized + "'");
  }

  VideoStreamSpec spec;
  spec.stream_key = makeVideoStreamKey(kTopicStreamKeyPrefix, normalized);
  spec.track_name = makeVideoTrackName(kTopicTrackNamePrefix, normalized);

  spec.ros_topic = normalized;
  spec.interface_type = interface_type;
  spec.input_kind = VideoInputKind::RosTopic;
  spec.ingest_mode = std::string(interface_classification->ingest_mode);

  spec.selected_config_id = matched_rule->rule_id;
  spec.transform_fragment = matched_rule->transform_fragment;
  spec.publish_config = matched_rule->publish_config;
  return spec;
}

VideoStreamSpec resolveConfiguredSourceVideoStreamSpec(
  const VideoStreamConfig & stream_config, const std::string & configured_source_name)
{
  const std::string trimmed_name = trimConfiguredSourceName(configured_source_name);
  if (trimmed_name.empty()) {
    throw std::invalid_argument("Invalid configured source name.");
  }

  const auto source_it = stream_config.configured_sources.find(trimmed_name);
  if (source_it == stream_config.configured_sources.end()) {
    throw std::invalid_argument("Unknown configured video source '" + trimmed_name + "'.");
  }

  const auto & configured_source = source_it->second;

  VideoStreamSpec spec;
  // stream_key, configured_source_name, and selected_config_id all use the same trimmed configured-source name.
  spec.stream_key = makeVideoStreamKey(kConfiguredSourceStreamKeyPrefix, trimmed_name);
  spec.track_name = makeConfiguredSourceTrackName(kConfiguredSourceTrackNamePrefix, trimmed_name);
  spec.configured_source_name = trimmed_name;
  spec.input_kind = VideoInputKind::ConfiguredSource;
  spec.selected_config_id = trimmed_name;

  spec.ingress_fragment = configured_source.ingress_fragment;
  spec.transform_fragment = configured_source.transform_fragment;
  spec.ingest_mode = kConfiguredSourceIngestMode;
  spec.publish_config = configured_source.publish_config;

  return spec;
}

}  // namespace livekit_ros2_bridge
