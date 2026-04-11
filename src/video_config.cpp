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

#include "video_config.hpp"

#include <stdexcept>

#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kTopicStreamKeyPrefix[] = "topic";
constexpr char kExternalStreamKeyPrefix[] = "external";
constexpr char kTopicTrackNamePrefix[] = "ros.video.";
constexpr char kExternalTrackNamePrefix[] = "ros.video.external.";

std::string makeVideoStreamKey(std::string_view prefix, const std::string & resource)
{
  return std::string(prefix) + ":" + resource;
}

std::string makeTrackSuffix(const std::string & resource)
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

std::string makeVideoTrackName(std::string_view prefix, const std::string & resource)
{
  const std::string suffix = makeTrackSuffix(resource);
  return suffix.empty() ? std::string(prefix) + "unnamed" : std::string(prefix) + suffix;
}

}  // namespace

VideoConfig makeDefaultVideoConfig()
{
  VideoConfig config;

  RosTopicRule default_rule;
  default_rule.pattern = "/*";
  default_rule.id = video_defaults::kDefaultRosProfileId;
  default_rule.transform = video_defaults::kDefaultRosTransform;
  default_rule.publish = config.publish;
  config.ros_topic_rules.push_back(std::move(default_rule));

  return config;
}

std::optional<RosVideoSourceClassification> classifyRosVideoInterfaceType(std::string_view interface_type)
{
  if (interface_type == kImageInterfaceType) {
    return RosVideoSourceClassification{kRawImageIngestMode};
  }
  if (interface_type == kCompressedImageInterfaceType) {
    return RosVideoSourceClassification{kCompressedImageIngestMode};
  }
  return std::nullopt;
}

std::string normalizeExternalName(std::string_view external_name)
{
  return normalizeRosResourceName(external_name);
}

std::string videoSourceKindToString(VideoSourceKind kind)
{
  if (kind == VideoSourceKind::RosTopic) {
    return "ros_topic";
  }
  return "external";
}

VideoStreamSpec resolveRosVideoStreamSpec(
  const VideoConfig & config, const std::string & topic, const std::string & interface_type)
{
  const std::string normalized = normalizeRosResourceName(topic);
  if (normalized.empty()) {
    throw std::invalid_argument("Invalid ROS topic.");
  }
  const auto source_classification = classifyRosVideoInterfaceType(interface_type);
  if (!source_classification.has_value()) {
    throw std::invalid_argument("ROS topic is not a supported video type.");
  }

  const RosTopicRule * matched_rule = nullptr;
  std::size_t best_len = 0;
  // Update only on a strictly longer pattern so same-length matches stay first-declared.
  for (const auto & rule : config.ros_topic_rules) {
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
  spec.source_kind = VideoSourceKind::RosTopic;
  spec.ingest_mode = std::string(source_classification->ingest_mode);
  spec.selected_config_key = matched_rule->id;
  spec.transform_description = matched_rule->transform;
  spec.publish_config = matched_rule->publish;
  return spec;
}

VideoStreamSpec resolveExternalVideoStreamSpec(const VideoConfig & config, const std::string & external_name)
{
  const std::string normalized = normalizeExternalName(external_name);
  if (normalized.empty()) {
    throw std::invalid_argument("Invalid external name.");
  }

  const auto source_it = config.external_sources.find(normalized);
  if (source_it == config.external_sources.end()) {
    throw std::invalid_argument("Unknown configured video source '" + normalized + "'.");
  }

  const auto & source = source_it->second;

  VideoStreamSpec spec;
  // stream_key, external_name, and selected_config_key all use the same canonical configured-source name.
  spec.stream_key = makeVideoStreamKey(kExternalStreamKeyPrefix, normalized);
  spec.track_name = makeVideoTrackName(kExternalTrackNamePrefix, normalized);
  spec.external_name = normalized;
  spec.source_kind = VideoSourceKind::External;
  spec.selected_config_key = normalized;
  spec.source_description = source.source;
  spec.transform_description = source.transform;
  spec.ingest_mode = kExternalIngestMode;
  spec.publish_config = source.publish;

  return spec;
}

}  // namespace livekit_ros2_bridge
