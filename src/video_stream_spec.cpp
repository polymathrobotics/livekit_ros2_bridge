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
#include <variant>

#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kTopicKeyPrefix[] = "topic";
constexpr char kOtherVideoKeyPrefix[] = "other_video";
constexpr char kTopicTrackPrefix[] = "lkros.video.";
constexpr char kOtherVideoTrackPrefix[] = "lkros.video.other.";
constexpr char kHexDigits[] = "0123456789ABCDEF";
const auto kLogger = rclcpp::get_logger("video_stream_spec");

// ROS-topic track names retain the lossy legacy mapping; stream_key carries exact identity.
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

bool isUnreservedTrackByte(unsigned char byte)
{
  return (byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z') || (byte >= '0' && byte <= '9') || byte == '-' ||
         byte == '.' || byte == '_' || byte == '~';
}

std::string encodeOtherVideoTrackSuffix(std::string_view name)
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

const RosVideoTopicRule & selectRosTopicRule(const std::vector<RosVideoTopicRule> & rules, std::string_view topic)
{
  const RosVideoTopicRule * match = nullptr;
  for (const auto & rule : rules) {
    if (!rosResourceMatchesPattern(topic, rule.pattern)) {
      continue;
    }
    const auto size = rule.pattern.size();
    if (match != nullptr && size <= match->pattern.size()) {
      continue;
    }
    match = &rule;
  }
  if (match == nullptr) {
    LogEvent(kLogger, "video_stream_spec_rejected")
      .field("resource", topic)
      .field("reason", "no_matching_ros_topic_rule")
      .warn();
    throw std::runtime_error("no matching video rule for topic '" + std::string(topic) + "'");
  }
  return *match;
}

}  // namespace

std::optional<RosVideoIngestMode> classifyRosVideoIngestMode(std::string_view interface_type)
{
  if (interface_type == rosidl_generator_traits::name<sensor_msgs::msg::Image>()) {
    return RosVideoIngestMode::RawImage;
  }
  if (interface_type == rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>()) {
    return RosVideoIngestMode::CompressedImage;
  }
  return std::nullopt;
}

const RosVideoInput & requireRosVideoInput(const VideoStreamSpec & spec)
{
  if (const auto * input = std::get_if<RosVideoInput>(&spec.input); input != nullptr) {
    return *input;
  }
  throw std::logic_error("Video stream spec does not contain a ROS video input.");
}

const OtherVideoInput & requireOtherVideoInput(const VideoStreamSpec & spec)
{
  if (const auto * input = std::get_if<OtherVideoInput>(&spec.input); input != nullptr) {
    return *input;
  }
  throw std::logic_error("Video stream spec does not contain an other-video input.");
}

VideoStreamSpec resolveRosVideoTopicSpec(
  const VideoStreamConfig & config, const std::string & requested_topic, const std::string & interface_type)
{
  const std::string topic = normalizeRosResourceName(requested_topic);
  if (topic.empty()) {
    throw std::invalid_argument("Invalid ROS topic.");
  }
  const auto ingest_mode = classifyRosVideoIngestMode(interface_type);
  if (!ingest_mode.has_value()) {
    LogEvent(kLogger, "video_stream_spec_rejected")
      .field("resource", topic)
      .fieldOr("interface_type", interface_type)
      .field("reason", "unsupported_ros_interface_type")
      .warn();
    throw std::invalid_argument("ROS topic is not a supported video type.");
  }

  const auto & rule = selectRosTopicRule(config.ros_topic_rules, topic);

  VideoStreamSpec spec;
  spec.stream_key = std::string{kTopicKeyPrefix} + ":" + topic;
  spec.track_name = std::string{kTopicTrackPrefix} + makeTopicTrackSuffix(topic);
  spec.input = RosVideoInput{
    topic,
    interface_type,
    *ingest_mode,
    rule.rule_id,
    rule.transform_fragment,
  };
  spec.publish_options = rule.publish_options;
  return spec;
}

VideoStreamSpec resolveOtherVideoSourceSpec(const VideoStreamConfig & config, const std::string & source_name)
{
  const std::string name = trim(source_name);
  if (name.empty()) {
    throw std::invalid_argument("Invalid other video name.");
  }

  const auto it = config.other_video_sources.find(name);
  if (it == config.other_video_sources.end()) {
    throw std::invalid_argument("Unknown other video source '" + name + "'.");
  }

  const auto & source = it->second;

  VideoStreamSpec spec;
  spec.stream_key = std::string{kOtherVideoKeyPrefix} + ":" + name;
  spec.track_name = std::string{kOtherVideoTrackPrefix} + encodeOtherVideoTrackSuffix(name);
  spec.input = OtherVideoInput{
    name,
    source.ingress_fragment,
    source.transform_fragment,
  };
  spec.publish_options = source.publish_options;

  return spec;
}

}  // namespace livekit_ros2_bridge
