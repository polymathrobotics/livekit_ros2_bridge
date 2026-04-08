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

#include <sstream>
#include <stdexcept>

#include "utils/interface_types.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::string makeVideoSidecarKey(std::string_view prefix, const std::string & resource)
{
  return std::string(prefix) + ":" + resource;
}

/// Map a ROS interface type to the pipeline alias used in PipelineMap keys.
std::string interfaceTypeToAlias(std::string_view interface_type)
{
  if (interface_type == kImageInterfaceType) {
    return "image";
  }
  if (interface_type == kCompressedImageInterfaceType) {
    return "compressed_image";
  }
  return "default";
}

/// Replace all occurrences of `{topic}` in the template with the given topic.
std::string interpolateTopic(const std::string & tmpl, const std::string & topic)
{
  static constexpr char kPlaceholder[] = "{topic}";
  static constexpr std::size_t kPlaceholderLen = sizeof(kPlaceholder) - 1;
  std::string result = tmpl;
  std::size_t pos = 0;
  while ((pos = result.find(kPlaceholder, pos)) != std::string::npos) {
    result.replace(pos, kPlaceholderLen, topic);
    pos += topic.size();
  }
  return result;
}

/// Tokenize a pipeline string by whitespace.
std::vector<std::string> tokenizePipeline(const std::string & pipeline)
{
  std::vector<std::string> tokens;
  std::istringstream stream(pipeline);
  std::string token;
  while (stream >> token) {
    tokens.push_back(std::move(token));
  }
  return tokens;
}

}  // namespace

VideoConfig makeDefaultVideoConfig()
{
  VideoConfig config;

  RosTopicRule default_rule;
  default_rule.pattern = "/*";
  default_rule.id = video_defaults::kDefaultRosProfileId;
  default_rule.pipelines = {
    {"image", video_defaults::kDefaultImagePipeline},
    {"compressed_image", video_defaults::kDefaultCompressedImagePipeline},
  };
  config.ros_topic_rules.push_back(std::move(default_rule));

  return config;
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
  return "pipeline";
}

SidecarLaunchSpec resolveRosVideoLaunchSpec(
  const VideoConfig & config, const std::string & topic, const std::string & interface_type)
{
  const std::string normalized = normalizeRosResourceName(topic);
  if (normalized.empty()) {
    throw std::invalid_argument("Invalid ROS topic.");
  }
  if (!isSupportedVideoInterfaceType(interface_type)) {
    throw std::invalid_argument("ROS topic is not a supported video type.");
  }

  const RosTopicRule * matched_rule = nullptr;
  std::size_t best_len = 0;
  for (const auto & rule : config.ros_topic_rules) {
    if (
      rosResourceMatchesPattern(normalized, rule.pattern) &&
      (matched_rule == nullptr || rule.pattern.size() > best_len))
    {
      matched_rule = &rule;
      best_len = rule.pattern.size();
    }
  }
  if (matched_rule == nullptr) {
    throw std::runtime_error("no matching video rule for topic '" + normalized + "'");
  }

  const std::string alias = interfaceTypeToAlias(interface_type);
  auto pipeline_it = matched_rule->pipelines.find(alias);
  if (pipeline_it == matched_rule->pipelines.end()) {
    pipeline_it = matched_rule->pipelines.find("default");
  }
  if (pipeline_it == matched_rule->pipelines.end()) {
    throw std::runtime_error("no pipeline for alias '" + alias + "' in video rule '" + matched_rule->pattern + "'");
  }

  const std::string resolved = interpolateTopic(pipeline_it->second, normalized);

  SidecarLaunchSpec spec;
  spec.sidecar_key = makeVideoSidecarKey("topic", normalized);
  spec.ros_topic = normalized;
  spec.interface_type = interface_type;
  spec.source_kind = VideoSourceKind::RosTopic;
  spec.ingest_mode = interface_type == kCompressedImageInterfaceType ? "compressed_image" : "raw_image";
  spec.selected_config_key = matched_rule->id;
  spec.source_pipeline = tokenizePipeline(resolved);
  return spec;
}

SidecarLaunchSpec resolvePipelineVideoLaunchSpec(const VideoConfig & config, const std::string & external_name)
{
  const std::string normalized = normalizeExternalName(external_name);
  if (normalized.empty()) {
    throw std::invalid_argument("Invalid external name.");
  }

  const auto source_it = config.pipeline_sources.find(normalized);
  if (source_it == config.pipeline_sources.end()) {
    throw std::invalid_argument("Unknown configured video source '" + normalized + "'.");
  }

  const auto & source = source_it->second;

  SidecarLaunchSpec spec;
  spec.sidecar_key = makeVideoSidecarKey("external", normalized);
  spec.external_name = normalized;
  spec.source_kind = VideoSourceKind::Pipeline;
  spec.selected_config_key = source.external_name;
  spec.source_pipeline = tokenizePipeline(source.pipeline);
  spec.ingest_mode = "pipeline";

  return spec;
}

}  // namespace livekit_ros2_bridge
