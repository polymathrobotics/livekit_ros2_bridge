// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "video/stream_config.hpp"

#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "livekit/room_event_types.h"
#include "livekit/video_codec.h"
#include "utils/gstreamer_pipeline_validation.hpp"
#include "utils/param_entries.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video/gstreamer_resources.hpp"
#include "video/pipeline_description.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

RosResourcePattern requireRosResourcePattern(std::string_view raw, const char * context)
{
  if (trim(raw).empty()) {
    throw std::runtime_error(std::string(context) + " pattern must not be empty");
  }
  auto pattern = RosResourcePattern::parse(raw);
  if (!pattern.has_value()) {
    throw std::runtime_error(std::string(context) + " pattern must normalize to a valid ROS resource");
  }
  return *pattern;
}

std::optional<livekit::VideoCodec> parseVideoCodec(const std::string & value)
{
  const std::string codec = trim(value);
  if (codec == "vp8") {
    return livekit::VideoCodec::VP8;
  }
  if (codec == "h264") {
    return livekit::VideoCodec::H264;
  }
  if (codec == "av1") {
    return livekit::VideoCodec::AV1;
  }
  if (codec == "vp9") {
    return livekit::VideoCodec::VP9;
  }
  if (codec == "h265") {
    return livekit::VideoCodec::H265;
  }
  return std::nullopt;
}

std::optional<bool> parseSimulcast(const std::string & value)
{
  const std::string simulcast = trim(value);
  if (simulcast == "enabled") {
    return true;
  }
  if (simulcast == "disabled") {
    return false;
  }
  return std::nullopt;
}

void setEncoding(livekit::TrackPublishOptions & options, std::uint64_t max_bitrate_bps, double max_framerate)
{
  if (max_bitrate_bps > 0 || max_framerate > 0.0) {
    livekit::VideoEncodingOptions encoding;
    encoding.max_bitrate = max_bitrate_bps;
    encoding.max_framerate = max_framerate;
    options.video_encoding = encoding;
    return;
  }

  options.video_encoding.reset();
}

livekit::TrackPublishOptions parsePublishOptions(const Params & params)
{
  livekit::TrackPublishOptions options;
  options.video_codec = parseVideoCodec(params.video.publish.codec);
  setEncoding(
    options, static_cast<std::uint64_t>(params.video.publish.max_bitrate_bps), params.video.publish.max_framerate);
  options.simulcast = parseSimulcast(params.video.publish.simulcast);
  return options;
}

template <typename EntryT>
livekit::TrackPublishOptions parsePublishOptions(const EntryT & entry, const livekit::TrackPublishOptions & defaults)
{
  livekit::TrackPublishOptions options = defaults;

  const std::string codec = trim(entry.publish.codec);
  if (!codec.empty()) {
    options.video_codec = parseVideoCodec(codec);
  }

  // Negative values mean "inherit global default"; the generated parameter
  // schema cannot express optional scalars for these fields.
  std::uint64_t max_bitrate_bps = 0;
  double max_framerate = 0.0;
  if (options.video_encoding.has_value()) {
    max_bitrate_bps = options.video_encoding->max_bitrate;
    max_framerate = options.video_encoding->max_framerate;
  }
  if (entry.publish.max_bitrate_bps >= 0) {
    max_bitrate_bps = static_cast<std::uint64_t>(entry.publish.max_bitrate_bps);
  }
  if (entry.publish.max_framerate >= 0.0) {
    max_framerate = entry.publish.max_framerate;
  }
  setEncoding(options, max_bitrate_bps, max_framerate);

  const std::string simulcast = trim(entry.publish.simulcast);
  if (!simulcast.empty()) {
    options.simulcast = parseSimulcast(simulcast);
  }

  return options;
}

// Passthrough forwards the camera's video unchanged and ignores global defaults. Returns the first
// per-entry setting that is set, or null, also for entries without passthrough. Only
// publish.max_bitrate_bps may be set, because it describes the camera.
template <typename EntryT>
const char * findPassthroughOverride(const EntryT & entry)
{
  if (!entry.passthrough) {
    return nullptr;
  }
  if (!trim(entry.transform).empty()) {
    return "transform";
  }
  if (!trim(entry.publish.codec).empty()) {
    return "publish.codec";
  }
  if (entry.publish.max_framerate >= 0.0) {
    return "publish.max_framerate";
  }
  if (!trim(entry.publish.simulcast).empty()) {
    return "publish.simulcast";
  }
  return nullptr;
}

// Leaves the codec unset: the publisher takes it from the camera's first keyframe. publish.max_bitrate_bps
// tells LiveKit the most this camera sends, which bounds its rate target and pacing; -1 (unset) and 0
// leave LiveKit's default for the resolution.
template <typename EntryT>
livekit::TrackPublishOptions makePassthroughPublishOptions(const EntryT & entry)
{
  livekit::TrackPublishOptions options;
  options.video_encoder = livekit::VideoEncoderBackend::PreEncoded;
  options.simulcast = false;
  if (entry.publish.max_bitrate_bps >= 0) {
    setEncoding(options, static_cast<std::uint64_t>(entry.publish.max_bitrate_bps), 0.0);
  }
  return options;
}

constexpr utils::EndpointLayout kRosTopicRuleLayout{1U, 1U, 1U, 1U, kBridgeAppSrcName, kBridgeAppSinkName};
constexpr utils::EndpointLayout kOtherSourceLayout = utils::makeOtherSourceLayout(kBridgeAppSinkName);

using utils::requireUniqueEntry;

}  // namespace

StreamConfig loadConfig(const Params & params)
{
  StreamConfig config = makeDefaultConfig();
  config.default_publish_options = parsePublishOptions(params);
  for (auto & rule : config.ros_topic_rules) {
    rule.publish_options = config.default_publish_options;
  }

  // User rules precede the built-in catch-all; same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_topic_ids;
  std::unordered_set<std::string> seen_source_ids;
  std::unordered_set<std::string> seen_source_names;
  // Validate topic transforms against bridge-owned source/sink endpoints.
  const std::string synthetic_appsrc = "appsrc name=" + std::string{kBridgeAppSrcName} +
                                       " is-live=true block=false format=time do-timestamp=true"
                                       " caps=video/x-raw,format=RGB,width=2,height=2,framerate=0/1";
  for (const auto & id : params.video_topic_ids) {
    const auto & entry =
      requireUniqueEntry(seen_topic_ids, id, params.video.topics.video_topic_ids_map, "video topic id", "video topic");

    const std::string rule_context = "video topic '" + id + "'";
    const RosResourcePattern pattern = requireRosResourcePattern(entry.pattern, "video topic");
    const std::string transform = trim(entry.transform);
    validatePipeline(
      rule_context + " transform", buildPipelineDescription(synthetic_appsrc, transform), kRosTopicRuleLayout);

    RosTopicRule rule;
    rule.pattern = pattern;
    rule.rule_id = id;
    rule.transform_fragment = transform;
    rule.publish_options = parsePublishOptions(entry, config.default_publish_options);
    config.ros_topic_rules.push_back(std::move(rule));
  }

  for (const auto & id : params.video_other_ids) {
    const auto & entry = requireUniqueEntry(
      seen_source_ids, id, params.video.other.video_other_ids_map, "other video id", "other video source");

    const std::string source_context = "other video source '" + id + "'";
    const std::string source_fragment = trim(entry.source);
    if (source_fragment.empty()) {
      throw std::runtime_error(source_context + " requires a non-empty source");
    }
    if (const char * setting = findPassthroughOverride(entry)) {
      throw std::runtime_error(
        source_context + " sets " + setting +
        ", but passthrough forwards the camera's video unchanged; leave it unset");
    }
    const std::string transform = trim(entry.transform);
    const std::string description = entry.passthrough ? buildPassthroughPipelineDescription(source_fragment)
                                                      : buildPipelineDescription(source_fragment, transform);
    validatePipeline(source_context, description, kOtherSourceLayout);

    // Only trim surrounding whitespace; slash and colon variants stay distinct.
    const std::string name = trim(id);
    if (name.empty()) {
      throw std::runtime_error(source_context + " must trim to a non-empty name");
    }
    if (!seen_source_names.emplace(name).second) {
      throw std::runtime_error("duplicate other video source name '" + name + "'");
    }

    OtherSource source;
    source.name = name;
    source.source_fragment = source_fragment;
    source.transform_fragment = transform;
    source.passthrough = entry.passthrough;
    source.publish_options = entry.passthrough ? makePassthroughPublishOptions(entry)
                                               : parsePublishOptions(entry, config.default_publish_options);
    config.other_sources.push_back(std::move(source));
  }

  config.ros_topic_rules.insert(
    config.ros_topic_rules.end(),
    std::make_move_iterator(builtin_rules.begin()),
    std::make_move_iterator(builtin_rules.end()));

  return config;
}

}  // namespace livekit_ros2_bridge::video
