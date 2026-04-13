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
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/gstreamer_raii.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRuntimeConfigLogger = rclcpp::get_logger("livekit_ros2_bridge.runtime_config");
constexpr char kUnsetLogValue[] = "<unset>";
constexpr char kBridgeVideoAppSrcName[] = "bridge_video_src";
constexpr char kBridgeVideoAppSinkName[] = "bridge_video_sink";

std::string normalizeRosResourcePattern(std::string_view raw_pattern, const char * context)
{
  const std::string trimmed = trim(raw_pattern);
  if (trimmed.empty()) {
    throw std::runtime_error(std::string(context) + " pattern must not be empty");
  }
  // Store patterns in the canonical absolute form used by downstream matching.
  // User-facing "*" is still accepted here and normalized to the catch-all "/*".
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

SubscriptionQosReliabilityMode parseSubscriptionQosReliability(const std::string & raw_mode)
{
  if (raw_mode == "auto") {
    return SubscriptionQosReliabilityMode::kAuto;
  }
  if (raw_mode == "reliable") {
    return SubscriptionQosReliabilityMode::kReliable;
  }
  if (raw_mode == "best_effort") {
    return SubscriptionQosReliabilityMode::kBestEffort;
  }

  throw std::runtime_error("unsupported subscription.qos_overrides reliability mode '" + raw_mode + "'");
}

SubscriptionQosDurabilityMode parseSubscriptionQosDurability(const std::string & raw_mode)
{
  if (raw_mode == "auto") {
    return SubscriptionQosDurabilityMode::kAuto;
  }
  if (raw_mode == "volatile") {
    return SubscriptionQosDurabilityMode::kVolatile;
  }
  if (raw_mode == "transient_local") {
    return SubscriptionQosDurabilityMode::kTransientLocal;
  }

  throw std::runtime_error("unsupported subscription.qos_overrides durability mode '" + raw_mode + "'");
}

VideoPublishCodec parseVideoPublishCodec(const std::string & raw_codec, const std::string & field_name)
{
  if (raw_codec == "auto") {
    return VideoPublishCodec::Auto;
  }
  if (raw_codec == "vp8") {
    return VideoPublishCodec::Vp8;
  }
  if (raw_codec == "h264") {
    return VideoPublishCodec::H264;
  }
  if (raw_codec == "av1") {
    return VideoPublishCodec::Av1;
  }
  if (raw_codec == "vp9") {
    return VideoPublishCodec::Vp9;
  }
  if (raw_codec == "h265") {
    return VideoPublishCodec::H265;
  }

  throw std::runtime_error("unsupported " + field_name + " '" + raw_codec + "'");
}

VideoPublishSimulcast parseVideoPublishSimulcast(const std::string & raw_simulcast, const std::string & field_name)
{
  if (raw_simulcast == "auto") {
    return VideoPublishSimulcast::Auto;
  }
  if (raw_simulcast == "enabled") {
    return VideoPublishSimulcast::Enabled;
  }
  if (raw_simulcast == "disabled") {
    return VideoPublishSimulcast::Disabled;
  }

  throw std::runtime_error("unsupported " + field_name + " '" + raw_simulcast + "'");
}

VideoPublishConfig parseVideoPublishConfig(const Params & params)
{
  VideoPublishConfig config;
  config.codec = parseVideoPublishCodec(trim(params.video.publish.codec), "video.publish.codec");
  config.max_bitrate_bps = static_cast<std::uint64_t>(params.video.publish.max_bitrate_bps);
  config.max_framerate = params.video.publish.max_framerate;
  config.simulcast = parseVideoPublishSimulcast(trim(params.video.publish.simulcast), "video.publish.simulcast");
  return config;
}

template <typename EntryT>
VideoPublishConfig parseVideoPublishConfig(
  const EntryT & entry, const std::string & context, const VideoPublishConfig & default_publish_config)
{
  VideoPublishConfig config = default_publish_config;

  const std::string codec = trim(entry.publish.codec);
  if (!codec.empty()) {
    config.codec = parseVideoPublishCodec(codec, context + " publish.codec");
  }

  // Entry-level numeric overrides use negative values as "inherit the global
  // default" sentinels because the generated parameter schema cannot express
  // optional scalars for these fields.
  if (entry.publish.max_bitrate_bps >= 0) {
    config.max_bitrate_bps = static_cast<std::uint64_t>(entry.publish.max_bitrate_bps);
  }
  if (entry.publish.max_framerate >= 0.0) {
    config.max_framerate = entry.publish.max_framerate;
  }

  const std::string simulcast = trim(entry.publish.simulcast);
  if (!simulcast.empty()) {
    config.simulcast = parseVideoPublishSimulcast(simulcast, context + " publish.simulcast");
  }

  return config;
}

std::string buildVideoPipelineDescription(const std::string & ingress_fragment, const std::string & transform_fragment)
{
  std::string pipeline = ingress_fragment;
  if (!transform_fragment.empty()) {
    pipeline += " ! ";
    pipeline += transform_fragment;
  }
  pipeline += " ! queue max-size-buffers=2 leaky=downstream";
  pipeline += " ! videoconvert";
  // Keep validation aligned with the runtime pipeline: the bridge-owned tail
  // always normalizes frames to I420 before handing them to LiveKit.
  pipeline += " ! video/x-raw,format=I420";
  pipeline += " ! appsink name=";
  pipeline += kBridgeVideoAppSinkName;
  pipeline += " sync=false drop=true max-buffers=1 emit-signals=false";
  return pipeline;
}

struct EndpointCounts
{
  guint appsrc_count = 0;
  guint appsink_count = 0;
  guint bridge_appsrc_count = 0;
  guint bridge_appsink_count = 0;
};

// Parse failures can leave bridge-owned endpoints partially instantiated, so
// validation needs both an exact layout check and a looser user-endpoint check.
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
constexpr EndpointLayout kConfiguredSourceEndpointLayout{};

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

    if (element_name == kBridgeVideoAppSrcName) {
      if (!is_appsrc) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeVideoAppSrcName + "'");
      }
      ++counts.bridge_appsrc_count;
    }
    if (element_name == kBridgeVideoAppSinkName) {
      if (!is_appsink) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeVideoAppSinkName + "'");
      }
      ++counts.bridge_appsink_count;
    }

    item.reset();
  }

  return counts;
}

void validateVideoPipelineDescription(
  const std::string & context, const std::string & pipeline_description, const EndpointLayout & layout)
{
  ensureGstreamerInitialized();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(pipeline_description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  // gst_parse_launch can return both an error and a partially built bin. Check
  // endpoint ownership first so appsrc/appsink misuse is reported even if
  // parsing later failed for a second reason in the same fragment.
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
  // The generated parameter library exposes entry ids and entry payload maps as
  // separate structures, so validate both declaration order and map presence
  // before reading a referenced entry.
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
  config.default_publish_config = parseVideoPublishConfig(params);
  for (auto & rule : config.ros_topic_rules) {
    rule.publish_config = config.default_publish_config;
  }

  // Rebuild user rules ahead of the built-in catch-all so longest-match selection
  // still works and same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_rule_ids;
  std::unordered_set<std::string> seen_source_ids;
  std::unordered_set<std::string> seen_source_names;
  // Topic-rule transforms are middle fragments only. Wrap them in a synthetic
  // bridge-owned ingress so validation exercises the same ownership shape the
  // runtime will assemble around a ROS subscription.
  const std::string validation_ingress = "appsrc name=" + std::string{kBridgeVideoAppSrcName} +
                                         " is-live=true block=false format=time do-timestamp=true"
                                         " caps=video/x-raw,format=RGB,width=2,height=2,framerate=0/1";
  for (const auto & entry_id : params.video_topic_rule_ids) {
    const auto & entry = requireUniqueEntry(
      seen_rule_ids,
      entry_id,
      params.video.topic_rules.video_topic_rule_ids_map,
      "video topic rule id",
      "video topic rule");

    const std::string rule_context = "video topic rule '" + entry_id + "'";
    const std::string pattern = normalizeRosResourcePattern(entry.pattern, "video topic rule");
    const std::string transform_fragment = trim(entry.transform);
    validateVideoPipelineDescription(
      rule_context + " transform",
      buildVideoPipelineDescription(validation_ingress, transform_fragment),
      kRosTopicRuleEndpointLayout);

    RosVideoTopicRule rule;
    rule.pattern = pattern;
    rule.rule_id = entry_id;
    rule.transform_fragment = transform_fragment;
    rule.publish_config = parseVideoPublishConfig(entry, rule_context, config.default_publish_config);
    config.ros_topic_rules.push_back(std::move(rule));
  }

  for (const auto & entry_id : params.video_configured_source_ids) {
    const auto & entry = requireUniqueEntry(
      seen_source_ids,
      entry_id,
      params.video.configured_sources.video_configured_source_ids_map,
      "video configured source id",
      "video configured source");

    const std::string source_context = "video configured source '" + entry_id + "'";
    const std::string ingress_fragment = trim(entry.source);
    if (ingress_fragment.empty()) {
      throw std::runtime_error(source_context + " requires a non-empty source");
    }
    const std::string transform_fragment = trim(entry.transform);
    validateVideoPipelineDescription(
      source_context,
      buildVideoPipelineDescription(ingress_fragment, transform_fragment),
      kConfiguredSourceEndpointLayout);

    // Configured sources are keyed by the trimmed configured-source name. Only
    // surrounding whitespace is ignored; slash and colon variants stay distinct.
    const std::string source_name = trim(entry_id);
    if (source_name.empty()) {
      throw std::runtime_error(
        "video configured source '" + entry_id + "' must trim to a non-empty configured source name");
    }
    if (!seen_source_names.emplace(source_name).second) {
      throw std::runtime_error("duplicate configured video source name '" + source_name + "'");
    }

    ConfiguredVideoStreamSource source;
    source.ingress_fragment = ingress_fragment;
    source.transform_fragment = transform_fragment;
    source.publish_config = parseVideoPublishConfig(entry, source_context, config.default_publish_config);
    config.configured_sources.emplace(source_name, std::move(source));
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
  for (const auto & override_id : params.subscription_qos_overrides_ids) {
    const auto & entry = requireUniqueEntry(
      seen_override_ids,
      override_id,
      params.subscription.qos_overrides.subscription_qos_overrides_ids_map,
      "subscription QoS override id",
      "subscription QoS override entry");

    TopicSubscriptionQosOverride topic_override;
    topic_override.id = override_id;
    topic_override.pattern = normalizeRosResourcePattern(entry.pattern, "subscription.qos_overrides");
    topic_override.reliability = parseSubscriptionQosReliability(entry.reliability);
    topic_override.durability = parseSubscriptionQosDurability(entry.durability);
    config.topic_overrides.push_back(std::move(topic_override));
  }

  return config;
}

// Keep the generated parameter-library field layout at the edge so startup
// config assembly reads in terms of runtime-owned concepts.
RoomConnectionConfig loadRoomConnectionConfig(const Params & params)
{
  if (params.livekit.url.empty()) {
    throw std::runtime_error("livekit.url is required");
  }
  if (params.livekit.room.empty()) {
    throw std::runtime_error("livekit.room is required");
  }

  RoomConnectionConfig config;
  config.url = params.livekit.url;
  config.room = params.livekit.room;
  return config;
}

RuntimeConfig::HealthConfig loadHealthConfig(const Params & params)
{
  RuntimeConfig::HealthConfig config;
  config.fail_fast_enabled = params.health.fail_fast.enabled;
  config.fail_fast_disconnect_grace = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(params.health.fail_fast.disconnect_grace_seconds));
  return config;
}

AccessPolicy loadAccessPolicy(const Params & params)
{
  AccessPolicyConfig config;
  config.publish = AccessRuleConfig{params.access.rules.publish.allow, params.access.rules.publish.deny};
  config.subscribe = AccessRuleConfig{params.access.rules.subscribe.allow, params.access.rules.subscribe.deny};
  config.service = AccessRuleConfig{params.access.rules.service.allow, params.access.rules.service.deny};
  return AccessPolicy(config);
}

VideoProfilingConfig loadVideoProfilingConfig(const Params & params)
{
  VideoProfilingConfig config;
  config.enabled = params.debug.video_profiling.enabled;
  config.summary_interval = std::chrono::milliseconds(params.debug.video_profiling.summary_interval_ms);
  config.trace_file = params.debug.video_profiling.trace_file;
  config.trace_max_events = static_cast<std::size_t>(params.debug.video_profiling.trace_max_events);
  return config;
}

}  // namespace

RuntimeConfig loadRuntimeConfig(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
{
  // Keep the last-known room and url outside the guarded load path so startup
  // failure logs can still identify which room was being configured.
  std::string room;
  std::string url;
  const char * stage = "parameters_interface_validation";
  const auto logLoadFailure = [&](const char * error) {
    LogEvent(kRuntimeConfigLogger, "runtime_config_load_failed")
      .field("stage", stage)
      .fieldOr("room", room, kUnsetLogValue)
      .fieldOr("url", url, kUnsetLogValue)
      .field("error", error)
      .error();
  };

  try {
    if (parameters == nullptr) {
      throw std::invalid_argument("parameters_interface is required");
    }

    stage = "parameter_snapshot";
    ParamListener listener(parameters);
    RuntimeConfig config;
    config.params = listener.get_params();
    const Params & params = config.params;
    room = params.livekit.room;
    url = params.livekit.url;

    stage = "room_connection_config";
    config.room_connection = loadRoomConnectionConfig(params);
    config.access_token = params.livekit.token;
    stage = "health_config";
    config.health = loadHealthConfig(params);
    stage = "access_policy";
    config.access_policy = loadAccessPolicy(params);
    stage = "subscription_qos_config";
    config.subscription_qos = loadSubscriptionQosConfig(params);
    stage = "video_stream_config";
    config.video_stream = loadVideoStreamConfig(params);
    stage = "video_profiling_config";
    config.video_profiling = loadVideoProfilingConfig(params);

    LogEvent(kRuntimeConfigLogger, "runtime_config_loaded")
      .fieldOr("room", config.room_connection.room, kUnsetLogValue)
      .fieldOr("url", config.room_connection.url, kUnsetLogValue)
      .field("custom_video_rule_count", params.video_topic_rule_ids.size())
      .field("configured_source_count", config.video_stream.configured_sources.size())
      .info();

    return config;
  } catch (const std::exception & exc) {
    logLoadFailure(exc.what());
    throw;
  } catch (...) {
    logLoadFailure("unknown_exception");
    throw;
  }
}

}  // namespace livekit_ros2_bridge
