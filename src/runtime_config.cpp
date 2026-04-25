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

#include "runtime_config.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "livekit/room_event_types.h"
#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/qos_string_conversions.h"
#include "subscription_qos.hpp"
#include "utils/gstreamer_raii.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_pipeline_description.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.runtime_config");
constexpr char kLivekitTokenEnvVar[] = "LIVEKIT_TOKEN";

std::string resolveLivekitAccessToken(const Params & params)
{
  if (!params.livekit.token.empty()) {
    return params.livekit.token;
  }

  const char * env_token = std::getenv(kLivekitTokenEnvVar);
  if (env_token != nullptr && env_token[0] != '\0') {
    return env_token;
  }

  throw std::runtime_error("LiveKit startup token is required; set livekit.token or LIVEKIT_TOKEN");
}

std::string normalizeRosResourcePattern(std::string_view raw_pattern, const char * context)
{
  const std::string trimmed = trim(raw_pattern);
  if (trimmed.empty()) {
    throw std::runtime_error(std::string(context) + " pattern must not be empty");
  }
  // Keep "*" as catch-all shorthand for the absolute matcher.
  if (trimmed == "*") {
    return "/*";
  }
  if (trimmed.size() >= 2 && trimmed.substr(trimmed.size() - 2) == "/*") {
    const std::string normalized = normalizeRosResourceName(trimmed.substr(0, trimmed.size() - 2));
    if (normalized.empty()) {
      throw std::runtime_error(std::string(context) + " pattern must normalize to a valid ROS resource");
    }
    return normalized + "/*";
  }

  const std::string normalized = normalizeRosResourceName(trimmed);
  if (normalized.empty()) {
    throw std::runtime_error(std::string(context) + " pattern must normalize to a valid ROS resource");
  }
  return normalized;
}

std::optional<livekit::VideoCodec> parseVideoPublishCodec(const std::string & raw_codec)
{
  const std::string codec = trim(raw_codec);
  // This SDK version lacks stable named constants; values match its wire mapping.
  if (codec == "vp8") {
    return static_cast<livekit::VideoCodec>(0);
  }
  if (codec == "h264") {
    return static_cast<livekit::VideoCodec>(1);
  }
  if (codec == "av1") {
    return static_cast<livekit::VideoCodec>(2);
  }
  if (codec == "vp9") {
    return static_cast<livekit::VideoCodec>(3);
  }
  if (codec == "h265") {
    return static_cast<livekit::VideoCodec>(4);
  }
  return std::nullopt;
}

std::optional<bool> parseVideoPublishSimulcast(const std::string & raw_simulcast)
{
  const std::string simulcast = trim(raw_simulcast);
  if (simulcast == "enabled") {
    return true;
  }
  if (simulcast == "disabled") {
    return false;
  }
  return std::nullopt;
}

void setVideoEncodingOptions(livekit::TrackPublishOptions & config, std::uint64_t max_bitrate_bps, double max_framerate)
{
  if (max_bitrate_bps > 0 || max_framerate > 0.0) {
    livekit::VideoEncodingOptions encoding;
    encoding.max_bitrate = max_bitrate_bps;
    encoding.max_framerate = max_framerate;
    config.video_encoding = encoding;
    return;
  }

  config.video_encoding.reset();
}

template <typename RosPolicy, typename ParseFn>
std::optional<RosPolicy> parseSubscriptionQosPolicy(const std::string & raw_mode, ParseFn parse)
{
  const std::string mode = trim(raw_mode);
  if (mode == "auto") {
    return std::nullopt;
  }

  const auto parsed = parse(mode.c_str());

  return static_cast<RosPolicy>(parsed);
}

livekit::TrackPublishOptions parseTrackPublishOptions(const Params & params)
{
  livekit::TrackPublishOptions config;
  config.video_codec = parseVideoPublishCodec(params.video.publish.codec);
  setVideoEncodingOptions(
    config, static_cast<std::uint64_t>(params.video.publish.max_bitrate_bps), params.video.publish.max_framerate);
  config.simulcast = parseVideoPublishSimulcast(params.video.publish.simulcast);
  return config;
}

template <typename EntryT>
livekit::TrackPublishOptions parseTrackPublishOptions(
  const EntryT & entry, const livekit::TrackPublishOptions & default_publish_config)
{
  livekit::TrackPublishOptions config = default_publish_config;

  const std::string codec = trim(entry.publish.codec);
  if (!codec.empty()) {
    config.video_codec = parseVideoPublishCodec(codec);
  }

  // Negative values mean "inherit global default"; the generated parameter
  // schema cannot express optional scalars for these fields.
  std::uint64_t max_bitrate_bps = 0;
  double max_framerate = 0.0;
  if (config.video_encoding.has_value()) {
    max_bitrate_bps = config.video_encoding->max_bitrate;
    max_framerate = config.video_encoding->max_framerate;
  }
  if (entry.publish.max_bitrate_bps >= 0) {
    max_bitrate_bps = static_cast<std::uint64_t>(entry.publish.max_bitrate_bps);
  }
  if (entry.publish.max_framerate >= 0.0) {
    max_framerate = entry.publish.max_framerate;
  }
  setVideoEncodingOptions(config, max_bitrate_bps, max_framerate);

  const std::string simulcast = trim(entry.publish.simulcast);
  if (!simulcast.empty()) {
    config.simulcast = parseVideoPublishSimulcast(simulcast);
  }

  return config;
}

struct EndpointCounts
{
  guint appsrc_count = 0;
  guint appsink_count = 0;
  guint bridge_appsrc_count = 0;
  guint bridge_appsink_count = 0;
};

struct EndpointLayout
{
  guint appsrc_count = 0;
  guint appsink_count = 1;
  guint bridge_appsrc_count = 0;
  guint bridge_appsink_count = 1;

  constexpr bool matches(const EndpointCounts & counts) const noexcept
  {
    return counts.appsrc_count == appsrc_count && counts.appsink_count == appsink_count &&
           counts.bridge_appsrc_count == bridge_appsrc_count && counts.bridge_appsink_count == bridge_appsink_count;
  }

  constexpr bool hasUserEndpoints(const EndpointCounts & counts) const noexcept
  {
    if (
      counts.appsrc_count > appsrc_count || counts.appsink_count > appsink_count ||
      counts.bridge_appsrc_count > bridge_appsrc_count || counts.bridge_appsink_count > bridge_appsink_count)
    {
      return true;
    }

    return (counts.appsrc_count != 0U && counts.bridge_appsrc_count == 0U) ||
           (counts.appsink_count != 0U && counts.bridge_appsink_count == 0U);
  }
};

constexpr EndpointLayout kRosTopicRuleEndpointLayout{1U, 1U, 1U, 1U};
constexpr EndpointLayout kOtherVideoSourceEndpointLayout{};

EndpointCounts countPipelineEndpoints(const std::string & context, GstElement * pipeline)
{
  EndpointCounts counts;

  GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline)));
  GValueGuard item;
  while (true) {
    const GstIteratorResult result = gst_iterator_next(iterator.get(), item.get());
    if (result == GST_ITERATOR_DONE) {
      break;
    }
    if (result == GST_ITERATOR_RESYNC) {
      gst_iterator_resync(iterator.get());
      continue;
    }
    if (result != GST_ITERATOR_OK) {
      throw std::runtime_error(context + " could not inspect parsed GStreamer elements");
    }

    auto * element = GST_ELEMENT(g_value_get_object(item.get()));
    const GstElementFactory * factory = gst_element_get_factory(element);
    const std::string_view factory_name = factory == nullptr ? "" : GST_OBJECT_NAME(factory);
    const std::string_view element_name = GST_ELEMENT_NAME(element);

    const bool is_appsrc = factory_name == "appsrc";
    const bool is_appsink = factory_name == "appsink";
    if (is_appsrc) {
      ++counts.appsrc_count;
    }
    if (is_appsink) {
      ++counts.appsink_count;
    }

    if (element_name == kBridgeAppSrcName) {
      if (!is_appsrc) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeAppSrcName + "'");
      }
      ++counts.bridge_appsrc_count;
    }
    if (element_name == kBridgeAppSinkName) {
      if (!is_appsink) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeAppSinkName + "'");
      }
      ++counts.bridge_appsink_count;
    }

    item.reset();
  }

  return counts;
}

void validateVideoPipelineDescription(
  const std::string & context, const std::string & description, const EndpointLayout & layout)
{
  ensureGstreamerInitialized();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  // Prefer the endpoint-ownership error when a partial parse already shows it.
  if (error != nullptr && pipeline != nullptr) {
    const EndpointCounts counts = countPipelineEndpoints(context, pipeline.get());
    if (layout.hasUserEndpoints(counts)) {
      throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
    }
  }
  if (error != nullptr) {
    throw std::runtime_error(context + " has invalid GStreamer syntax: " + error->message);
  }
  if (pipeline == nullptr) {
    throw std::runtime_error(context + " has invalid GStreamer syntax: gst_parse_launch returned null");
  }

  if (!GST_IS_BIN(pipeline.get())) {
    throw std::runtime_error(context + " must parse to a GstBin");
  }

  const EndpointCounts counts = countPipelineEndpoints(context, pipeline.get());
  if (!layout.matches(counts)) {
    throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
  }
}

template <typename EntryMap>
const typename EntryMap::mapped_type & requireUniqueEntry(
  std::unordered_set<std::string> & seen_ids,
  const std::string & entry_id,
  const EntryMap & entries,
  const char * duplicate_context,
  const char * missing_context)
{
  if (!seen_ids.emplace(entry_id).second) {
    throw std::runtime_error(std::string("duplicate ") + duplicate_context + " '" + entry_id + "'");
  }

  const auto it = entries.find(entry_id);
  if (it == entries.end()) {
    throw std::runtime_error(std::string(missing_context) + " '" + entry_id + "' is missing generated parameters");
  }

  return it->second;
}

}  // namespace

VideoStreamConfig loadVideoStreamConfig(const Params & params)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  config.default_publish_config = parseTrackPublishOptions(params);
  for (auto & rule : config.ros_topic_rules) {
    rule.publish_config = config.default_publish_config;
  }

  // User rules precede the built-in catch-all; same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_topic_ids;
  std::unordered_set<std::string> seen_other_ids;
  std::unordered_set<std::string> seen_other_names;
  // Validate topic transforms against bridge-owned ingress/egress endpoints.
  const std::string synthetic_ingress = "appsrc name=" + std::string{kBridgeAppSrcName} +
                                        " is-live=true block=false format=time do-timestamp=true"
                                        " caps=video/x-raw,format=RGB,width=2,height=2,framerate=0/1";
  for (const auto & entry_id : params.video_topic_ids) {
    const auto & entry = requireUniqueEntry(
      seen_topic_ids, entry_id, params.video.topics.video_topic_ids_map, "video topic id", "video topic");

    const std::string rule_context = "video topic '" + entry_id + "'";
    const std::string pattern = normalizeRosResourcePattern(entry.pattern, "video topic");
    const std::string transform = trim(entry.transform);
    validateVideoPipelineDescription(
      rule_context + " transform", buildPipelineDescription(synthetic_ingress, transform), kRosTopicRuleEndpointLayout);

    RosVideoTopicRule rule;
    rule.pattern = pattern;
    rule.rule_id = entry_id;
    rule.transform_fragment = transform;
    rule.publish_config = parseTrackPublishOptions(entry, config.default_publish_config);
    config.ros_topic_rules.push_back(std::move(rule));
  }

  for (const auto & entry_id : params.video_other_ids) {
    const auto & entry = requireUniqueEntry(
      seen_other_ids, entry_id, params.video.other.video_other_ids_map, "other video id", "other video source");

    const std::string source_context = "other video source '" + entry_id + "'";
    const std::string ingress = trim(entry.source);
    if (ingress.empty()) {
      throw std::runtime_error(source_context + " requires a non-empty source");
    }
    const std::string transform = trim(entry.transform);
    validateVideoPipelineDescription(
      source_context, buildPipelineDescription(ingress, transform), kOtherVideoSourceEndpointLayout);

    // Only trim surrounding whitespace; slash and colon variants stay distinct.
    const std::string other_video_source_name = trim(entry_id);
    if (other_video_source_name.empty()) {
      throw std::runtime_error(source_context + " must trim to a non-empty name");
    }
    if (!seen_other_names.emplace(other_video_source_name).second) {
      throw std::runtime_error("duplicate other video source name '" + other_video_source_name + "'");
    }

    OtherVideoSource source;
    source.ingress_fragment = ingress;
    source.transform_fragment = transform;
    source.publish_config = parseTrackPublishOptions(entry, config.default_publish_config);
    config.other_video_sources.emplace(other_video_source_name, std::move(source));
  }

  config.ros_topic_rules.insert(
    config.ros_topic_rules.end(),
    std::make_move_iterator(builtin_rules.begin()),
    std::make_move_iterator(builtin_rules.end()));

  return config;
}

namespace
{

SubscriptionQosConfig loadSubscriptionQosConfig(const Params & params)
{
  SubscriptionQosConfig config;

  std::unordered_set<std::string> seen_override_ids;
  for (const auto & override_id : params.subscription_qos_ids) {
    const auto & entry = requireUniqueEntry(
      seen_override_ids,
      override_id,
      params.subscription.qos.subscription_qos_ids_map,
      "subscription QoS override id",
      "subscription QoS override entry");

    TopicSubscriptionQosOverride topic_override;
    topic_override.id = override_id;
    topic_override.pattern = normalizeRosResourcePattern(entry.pattern, "subscription.qos");
    topic_override.reliability =
      parseSubscriptionQosPolicy<rclcpp::ReliabilityPolicy>(entry.reliability, rmw_qos_reliability_policy_from_str);
    topic_override.durability =
      parseSubscriptionQosPolicy<rclcpp::DurabilityPolicy>(entry.durability, rmw_qos_durability_policy_from_str);
    config.topic_overrides.push_back(std::move(topic_override));
  }

  return config;
}

}  // namespace

RuntimeConfig loadRuntimeConfig(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
{
  const char * stage = "parameters_interface_validation";

  try {
    if (parameters == nullptr) {
      throw std::invalid_argument("parameters_interface is required");
    }

    stage = "parameter_snapshot";
    ParamListener listener(parameters);
    const Params params = listener.get_params();

    RuntimeConfig config;
    stage = "livekit_config";
    config.livekit.url = params.livekit.url;
    config.livekit.access_token = resolveLivekitAccessToken(params);

    stage = "health_config";
    config.health.watchdog_enabled = params.health.watchdog.enabled;
    config.health.watchdog_recovery_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(params.health.watchdog.recovery_timeout_seconds));

    stage = "access_policy";
    config.access_policy = AccessPolicy(
      AccessPolicyConfig{
        AccessRuleConfig{params.access.rules.publish.allow, params.access.rules.publish.deny},
        AccessRuleConfig{params.access.rules.subscribe.allow, params.access.rules.subscribe.deny},
        AccessRuleConfig{params.access.rules.service.allow, params.access.rules.service.deny},
      });

    stage = "subscription_qos_config";
    config.subscription_qos = loadSubscriptionQosConfig(params);

    stage = "video_stream_config";
    config.video_stream = loadVideoStreamConfig(params);

    return config;
  } catch (...) {
    LogEvent(kLogger, "runtime_config_load_failed")
      .field("stage", stage)
      .fieldException("error", std::current_exception())
      .error();

    throw;
  }
}

}  // namespace livekit_ros2_bridge
