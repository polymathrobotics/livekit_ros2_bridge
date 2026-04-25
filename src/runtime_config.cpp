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

std::string resolveAccessToken(const Params & params)
{
  if (!params.livekit.token.empty()) {
    return params.livekit.token;
  }

  const char * value = std::getenv(kLivekitTokenEnvVar);
  if (value != nullptr && value[0] != '\0') {
    return value;
  }

  throw std::runtime_error("LiveKit startup token is required; set livekit.token or LIVEKIT_TOKEN");
}

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

void setVideoEncoding(livekit::TrackPublishOptions & options, std::uint64_t max_bitrate_bps, double max_framerate)
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

template <typename RosPolicy, typename ParseFn>
std::optional<RosPolicy> parseSubscriptionQosPolicy(const std::string & value, ParseFn parse)
{
  const std::string mode = trim(value);
  if (mode == "auto") {
    return std::nullopt;
  }

  const auto parsed = parse(mode.c_str());

  return static_cast<RosPolicy>(parsed);
}

livekit::TrackPublishOptions parseTrackPublishOptions(const Params & params)
{
  livekit::TrackPublishOptions options;
  options.video_codec = parseVideoCodec(params.video.publish.codec);
  setVideoEncoding(
    options, static_cast<std::uint64_t>(params.video.publish.max_bitrate_bps), params.video.publish.max_framerate);
  options.simulcast = parseSimulcast(params.video.publish.simulcast);
  return options;
}

template <typename EntryT>
livekit::TrackPublishOptions parseTrackPublishOptions(
  const EntryT & entry, const livekit::TrackPublishOptions & defaults)
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
  setVideoEncoding(options, max_bitrate_bps, max_framerate);

  const std::string simulcast = trim(entry.publish.simulcast);
  if (!simulcast.empty()) {
    options.simulcast = parseSimulcast(simulcast);
  }

  return options;
}

struct EndpointCounts
{
  guint appsrc = 0;
  guint appsink = 0;
  guint bridge_appsrc = 0;
  guint bridge_appsink = 0;
};

struct EndpointLayout
{
  guint appsrc = 0;
  guint appsink = 1;
  guint bridge_appsrc = 0;
  guint bridge_appsink = 1;

  constexpr bool matches(const EndpointCounts & counts) const noexcept
  {
    return counts.appsrc == appsrc && counts.appsink == appsink && counts.bridge_appsrc == bridge_appsrc &&
           counts.bridge_appsink == bridge_appsink;
  }

  constexpr bool hasUserEndpoint(const EndpointCounts & counts) const noexcept
  {
    if (
      counts.appsrc > appsrc || counts.appsink > appsink || counts.bridge_appsrc > bridge_appsrc ||
      counts.bridge_appsink > bridge_appsink)
    {
      return true;
    }

    return (counts.appsrc != 0U && counts.bridge_appsrc == 0U) || (counts.appsink != 0U && counts.bridge_appsink == 0U);
  }
};

constexpr EndpointLayout kRosTopicRuleLayout{1U, 1U, 1U, 1U};
constexpr EndpointLayout kOtherVideoSourceLayout{};

EndpointCounts countEndpoints(const std::string & context, GstElement * pipeline)
{
  EndpointCounts counts;

  GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline)));
  GValueSlot item;
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
      ++counts.appsrc;
    }
    if (is_appsink) {
      ++counts.appsink;
    }

    if (element_name == kBridgeAppSrcName) {
      if (!is_appsrc) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeAppSrcName + "'");
      }
      ++counts.bridge_appsrc;
    }
    if (element_name == kBridgeAppSinkName) {
      if (!is_appsink) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeAppSinkName + "'");
      }
      ++counts.bridge_appsink;
    }

    item.reset();
  }

  return counts;
}

void validatePipeline(const std::string & context, const std::string & description, const EndpointLayout & layout)
{
  ensureGStreamerInitialized();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  // Prefer the endpoint-ownership error when a partial parse already shows it.
  if (error != nullptr && pipeline != nullptr) {
    const EndpointCounts counts = countEndpoints(context, pipeline.get());
    if (layout.hasUserEndpoint(counts)) {
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

  const EndpointCounts counts = countEndpoints(context, pipeline.get());
  if (!layout.matches(counts)) {
    throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
  }
}

template <typename EntryMap>
const typename EntryMap::mapped_type & requireUniqueEntry(
  std::unordered_set<std::string> & seen,
  const std::string & id,
  const EntryMap & entries,
  const char * duplicate_label,
  const char * missing_label)
{
  if (!seen.emplace(id).second) {
    throw std::runtime_error(std::string("duplicate ") + duplicate_label + " '" + id + "'");
  }

  const auto it = entries.find(id);
  if (it == entries.end()) {
    throw std::runtime_error(std::string(missing_label) + " '" + id + "' is missing generated parameters");
  }

  return it->second;
}

}  // namespace

VideoStreamConfig loadVideoStreamConfig(const Params & params)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  config.default_publish_options = parseTrackPublishOptions(params);
  for (auto & rule : config.ros_topic_rules) {
    rule.publish_options = config.default_publish_options;
  }

  // User rules precede the built-in catch-all; same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_topics;
  std::unordered_set<std::string> seen_other;
  std::unordered_set<std::string> seen_names;
  // Validate topic transforms against bridge-owned source/sink endpoints.
  const std::string synthetic_source = "appsrc name=" + std::string{kBridgeAppSrcName} +
                                       " is-live=true block=false format=time do-timestamp=true"
                                       " caps=video/x-raw,format=RGB,width=2,height=2,framerate=0/1";
  for (const auto & id : params.video_topic_ids) {
    const auto & entry =
      requireUniqueEntry(seen_topics, id, params.video.topics.video_topic_ids_map, "video topic id", "video topic");

    const std::string rule_context = "video topic '" + id + "'";
    const RosResourcePattern pattern = requireRosResourcePattern(entry.pattern, "video topic");
    const std::string transform = trim(entry.transform);
    validatePipeline(
      rule_context + " transform", buildPipelineDescription(synthetic_source, transform), kRosTopicRuleLayout);

    RosVideoTopicRule rule;
    rule.pattern = pattern;
    rule.rule_id = id;
    rule.transform_fragment = transform;
    rule.publish_options = parseTrackPublishOptions(entry, config.default_publish_options);
    config.ros_topic_rules.push_back(std::move(rule));
  }

  for (const auto & id : params.video_other_ids) {
    const auto & entry = requireUniqueEntry(
      seen_other, id, params.video.other.video_other_ids_map, "other video id", "other video source");

    const std::string source_context = "other video source '" + id + "'";
    const std::string source_fragment = trim(entry.source);
    if (source_fragment.empty()) {
      throw std::runtime_error(source_context + " requires a non-empty source");
    }
    const std::string transform = trim(entry.transform);
    validatePipeline(source_context, buildPipelineDescription(source_fragment, transform), kOtherVideoSourceLayout);

    // Only trim surrounding whitespace; slash and colon variants stay distinct.
    const std::string name = trim(id);
    if (name.empty()) {
      throw std::runtime_error(source_context + " must trim to a non-empty name");
    }
    if (!seen_names.emplace(name).second) {
      throw std::runtime_error("duplicate other video source name '" + name + "'");
    }

    OtherVideoSource source;
    source.ingress_fragment = source_fragment;
    source.transform_fragment = transform;
    source.publish_options = parseTrackPublishOptions(entry, config.default_publish_options);
    config.other_video_sources.emplace(name, std::move(source));
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

  std::unordered_set<std::string> seen;
  for (const auto & id : params.subscription_qos_ids) {
    const auto & entry = requireUniqueEntry(
      seen,
      id,
      params.subscription.qos.subscription_qos_ids_map,
      "subscription QoS override id",
      "subscription QoS override entry");

    TopicSubscriptionQosOverride qos_override;
    qos_override.id = id;
    qos_override.pattern = requireRosResourcePattern(entry.pattern, "subscription.qos");
    qos_override.reliability =
      parseSubscriptionQosPolicy<rclcpp::ReliabilityPolicy>(entry.reliability, rmw_qos_reliability_policy_from_str);
    qos_override.durability =
      parseSubscriptionQosPolicy<rclcpp::DurabilityPolicy>(entry.durability, rmw_qos_durability_policy_from_str);
    config.topic_overrides.push_back(std::move(qos_override));
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
    config.livekit.access_token = resolveAccessToken(params);

    stage = "health_config";
    config.watchdog.enabled = params.health.watchdog.enabled;
    config.watchdog.recovery_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(params.health.watchdog.recovery_timeout_seconds));

    stage = "access_policy";
    config.access_policy = AccessPolicy(
      AccessPolicyConfig{
        AccessRulesConfig{params.access.rules.publish.allow, params.access.rules.publish.deny},
        AccessRulesConfig{params.access.rules.subscribe.allow, params.access.rules.subscribe.deny},
        AccessRulesConfig{params.access.rules.service.allow, params.access.rules.service.deny},
      });

    stage = "subscription_qos_config";
    config.subscription_qos = loadSubscriptionQosConfig(params);

    stage = "video_config";
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
