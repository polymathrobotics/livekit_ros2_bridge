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

#include <gst/gst.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRuntimeConfigLogger = rclcpp::get_logger("livekit_ros2_bridge.runtime_config");
constexpr char kUnsetLogValue[] = "<unset>";
constexpr char kBridgeVideoAppSrcName[] = "bridge_video_src";
constexpr char kBridgeVideoAppSinkName[] = "bridge_video_sink";

std::string deriveDefaultIdentity(const std::string & node_name)
{
  char hostname[256];
  hostname[0] = '\0';
  if (gethostname(hostname, sizeof(hostname)) != 0 || hostname[0] == '\0') {
    return node_name;
  }
  hostname[sizeof(hostname) - 1U] = '\0';
  return node_name + "-" + hostname;
}

void ensureGstreamerInitialized()
{
  static std::once_flag once;
  std::call_once(once, []() { gst_init(nullptr, nullptr); });
}

RoomConnectionConfig loadConnectConfig(const Params & params, const std::string & node_name)
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
  config.identity = params.livekit.identity.empty() ? deriveDefaultIdentity(node_name) : params.livekit.identity;
  return config;
}

enum class LiveKitAuthMode
{
  Missing,
  InvalidApiCredentialPair,
  StaticToken,
  StaticTokenWithApiCredentials,
  ApiKeyAndSecret,
};

const char * authModeString(const LiveKitAuthMode auth_mode)
{
  switch (auth_mode) {
    case LiveKitAuthMode::Missing:
      return "missing";
    case LiveKitAuthMode::InvalidApiCredentialPair:
      return "invalid_api_credential_pair";
    case LiveKitAuthMode::StaticToken:
      return "static_token";
    case LiveKitAuthMode::StaticTokenWithApiCredentials:
      return "static_token_with_api_credentials";
    case LiveKitAuthMode::ApiKeyAndSecret:
      return "api_key_and_secret";
  }

  return "unknown";
}

LiveKitAuthMode classifyLiveKitAuthMode(const Params & params)
{
  const bool has_api_key = !params.livekit.api_key.empty();
  const bool has_api_secret = !params.livekit.api_secret.empty();
  if (has_api_key != has_api_secret) {
    return LiveKitAuthMode::InvalidApiCredentialPair;
  }
  if (!params.livekit.token.empty()) {
    return has_api_key ? LiveKitAuthMode::StaticTokenWithApiCredentials : LiveKitAuthMode::StaticToken;
  }
  return has_api_key ? LiveKitAuthMode::ApiKeyAndSecret : LiveKitAuthMode::Missing;
}

bool hasValidLiveKitApiCredentialPair(const LiveKitAuthMode auth_mode)
{
  return auth_mode != LiveKitAuthMode::InvalidApiCredentialPair;
}

bool usesStaticToken(const LiveKitAuthMode auth_mode)
{
  return auth_mode == LiveKitAuthMode::StaticToken || auth_mode == LiveKitAuthMode::StaticTokenWithApiCredentials;
}

bool enablesApiMintedTokens(const LiveKitAuthMode auth_mode)
{
  return auth_mode == LiveKitAuthMode::StaticTokenWithApiCredentials || auth_mode == LiveKitAuthMode::ApiKeyAndSecret;
}

void validateLiveKitApiCredentialPair(const LiveKitAuthMode auth_mode)
{
  if (!hasValidLiveKitApiCredentialPair(auth_mode)) {
    throw std::runtime_error("livekit.api_key and livekit.api_secret must be both set or both unset");
  }
}

void validateTokenTtl(const Params & params, const LiveKitAuthMode auth_mode)
{
  if (enablesApiMintedTokens(auth_mode) && params.livekit.token_ttl_seconds <= 0) {
    throw std::runtime_error(
      "livekit.token_ttl_seconds must be > 0 when livekit.api_key/livekit.api_secret mint bridge tokens");
  }
}

std::shared_ptr<AccessTokenSource> loadTokenSource(const Params & params, const LiveKitAuthMode auth_mode)
{
  validateLiveKitApiCredentialPair(auth_mode);

  if (usesStaticToken(auth_mode)) {
    return std::make_shared<StaticTokenSource>(params.livekit.token);
  }

  if (auth_mode == LiveKitAuthMode::ApiKeyAndSecret) {
    return std::make_shared<ApiKeyAccessTokenSource>(
      params.livekit.api_key, params.livekit.api_secret, std::chrono::seconds(params.livekit.token_ttl_seconds));
  }

  throw std::runtime_error("Either livekit.token or livekit.api_key + livekit.api_secret must be set");
}

std::string normalizeRosTopicPattern(std::string_view raw_pattern, const char * context)
{
  const std::string trimmed = trim(raw_pattern);
  if (trimmed.empty()) {
    throw std::runtime_error(std::string(context) + " pattern must not be empty");
  }
  if (trimmed == "*") {
    return "/*";
  }
  const bool is_subtree_rule = trimmed.size() >= 2 && trimmed.substr(trimmed.size() - 2) == "/*";
  if (is_subtree_rule) {
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

std::string normalizeVideoRulePattern(std::string_view raw_pattern)
{
  return normalizeRosTopicPattern(raw_pattern, "video topic rule");
}

SubscriptionQosReliabilityMode parseSubscriptionQosReliabilityMode(const std::string & raw_mode)
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

  throw std::runtime_error("unsupported subscribe.qos_overrides reliability mode '" + raw_mode + "'");
}

SubscriptionQosDurabilityMode parseSubscriptionQosDurabilityMode(const std::string & raw_mode)
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

  throw std::runtime_error("unsupported subscribe.qos_overrides durability mode '" + raw_mode + "'");
}

VideoPublishCodec parseVideoPublishCodec(const std::string & raw_codec)
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

  throw std::runtime_error("unsupported video.publish.codec '" + raw_codec + "'");
}

VideoPublishSimulcast parseVideoPublishSimulcast(const std::string & raw_simulcast)
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

  throw std::runtime_error("unsupported video.publish.simulcast '" + raw_simulcast + "'");
}

struct VideoPublishOverride
{
  std::optional<VideoPublishCodec> codec;
  std::optional<std::uint64_t> max_bitrate_bps;
  std::optional<double> max_framerate;
  std::optional<VideoPublishSimulcast> simulcast;
};

VideoPublishCodec parseVideoPublishCodec(const std::string & raw_codec, const std::string & field_name)
{
  try {
    return parseVideoPublishCodec(raw_codec);
  } catch (const std::runtime_error &) {
    throw std::runtime_error("unsupported " + field_name + " '" + raw_codec + "'");
  }
}

VideoPublishSimulcast parseVideoPublishSimulcast(const std::string & raw_simulcast, const std::string & field_name)
{
  try {
    return parseVideoPublishSimulcast(raw_simulcast);
  } catch (const std::runtime_error &) {
    throw std::runtime_error("unsupported " + field_name + " '" + raw_simulcast + "'");
  }
}

std::optional<VideoPublishCodec> parseOptionalVideoPublishCodec(
  const std::string & raw_codec, const std::string & field_name)
{
  const std::string codec = trim(raw_codec);
  if (codec.empty()) {
    return std::nullopt;
  }
  return parseVideoPublishCodec(codec, field_name);
}

std::optional<VideoPublishSimulcast> parseOptionalVideoPublishSimulcast(
  const std::string & raw_simulcast, const std::string & field_name)
{
  const std::string simulcast = trim(raw_simulcast);
  if (simulcast.empty()) {
    return std::nullopt;
  }
  return parseVideoPublishSimulcast(simulcast, field_name);
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
VideoPublishOverride parseVideoPublishOverride(const EntryT & entry, const std::string & context)
{
  VideoPublishOverride config;

  if (
    const auto codec = parseOptionalVideoPublishCodec(entry.publish.codec, context + " publish.codec");
    codec.has_value())
  {
    config.codec = *codec;
  }
  if (entry.publish.max_bitrate_bps >= 0) {
    config.max_bitrate_bps = static_cast<std::uint64_t>(entry.publish.max_bitrate_bps);
  }
  if (entry.publish.max_framerate >= 0.0) {
    config.max_framerate = entry.publish.max_framerate;
  }
  if (
    const auto simulcast = parseOptionalVideoPublishSimulcast(entry.publish.simulcast, context + " publish.simulcast");
    simulcast.has_value())
  {
    config.simulcast = *simulcast;
  }

  return config;
}

VideoPublishConfig mergeVideoPublishConfig(const VideoPublishConfig & defaults, const VideoPublishOverride & override)
{
  VideoPublishConfig merged = defaults;
  if (override.codec.has_value()) {
    merged.codec = *override.codec;
  }
  if (override.max_bitrate_bps.has_value()) {
    merged.max_bitrate_bps = *override.max_bitrate_bps;
  }
  if (override.max_framerate.has_value()) {
    merged.max_framerate = *override.max_framerate;
  }
  if (override.simulcast.has_value()) {
    merged.simulcast = *override.simulcast;
  }
  return merged;
}

std::string composeVideoPipelineDescription(const std::string & prefix, const std::string & transform)
{
  std::string pipeline = prefix;
  if (!transform.empty()) {
    pipeline += " ! ";
    pipeline += transform;
  }
  pipeline += " ! queue max-size-buffers=2 leaky=downstream";
  pipeline += " ! videoconvert";
  pipeline += " ! video/x-raw,format=RGBA";
  pipeline += " ! appsink name=";
  pipeline += kBridgeVideoAppSinkName;
  pipeline += " sync=false drop=true max-buffers=1 emit-signals=false";
  return pipeline;
}

void validateBridgeManagedEndpoints(const std::string & context, GstElement * pipeline, bool expect_bridge_appsrc)
{
  guint appsrc_count = 0;
  guint appsink_count = 0;
  guint named_bridge_appsrc_count = 0;
  guint named_bridge_appsink_count = 0;

  GstIterator * iterator = gst_bin_iterate_recurse(GST_BIN(pipeline));
  GValue item = G_VALUE_INIT;
  while (true) {
    const GstIteratorResult result = gst_iterator_next(iterator, &item);
    if (result == GST_ITERATOR_DONE) {
      break;
    }
    if (result == GST_ITERATOR_RESYNC) {
      gst_iterator_resync(iterator);
      continue;
    }
    if (result != GST_ITERATOR_OK) {
      g_value_unset(&item);
      gst_iterator_free(iterator);
      throw std::runtime_error(context + " could not inspect parsed GStreamer elements");
    }

    auto * element = GST_ELEMENT(g_value_get_object(&item));
    const GstElementFactory * factory = gst_element_get_factory(element);
    const std::string_view factory_name = factory == nullptr ? "" : GST_OBJECT_NAME(factory);
    const std::string_view element_name = GST_ELEMENT_NAME(element);

    const bool is_appsrc = factory_name == "appsrc";
    const bool is_appsink = factory_name == "appsink";
    if (is_appsrc) {
      ++appsrc_count;
    }
    if (is_appsink) {
      ++appsink_count;
    }

    if (element_name == kBridgeVideoAppSrcName) {
      if (!is_appsrc) {
        g_value_unset(&item);
        gst_iterator_free(iterator);
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeVideoAppSrcName + "'");
      }
      ++named_bridge_appsrc_count;
    }
    if (element_name == kBridgeVideoAppSinkName) {
      if (!is_appsink) {
        g_value_unset(&item);
        gst_iterator_free(iterator);
        throw std::runtime_error(context + " must not reuse reserved element name '" + kBridgeVideoAppSinkName + "'");
      }
      ++named_bridge_appsink_count;
    }

    g_value_reset(&item);
  }
  g_value_unset(&item);
  gst_iterator_free(iterator);

  if (expect_bridge_appsrc) {
    if (
      appsrc_count != 1U || named_bridge_appsrc_count != 1U || appsink_count != 1U || named_bridge_appsink_count != 1U)
    {
      throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
    }
    return;
  }

  if (appsrc_count != 0U || appsink_count != 1U || named_bridge_appsink_count != 1U) {
    throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
  }
}

void validatePipelineDescription(
  const std::string & context, const std::string & pipeline_description, bool expect_bridge_appsrc)
{
  ensureGstreamerInitialized();

  GError * error = nullptr;
  GstElement * pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
  if (pipeline == nullptr) {
    const std::string message = error != nullptr ? error->message : "gst_parse_launch returned null";
    if (error != nullptr) {
      g_error_free(error);
    }
    throw std::runtime_error(context + " has invalid GStreamer syntax: " + message);
  }

  if (!GST_IS_BIN(pipeline)) {
    gst_object_unref(pipeline);
    throw std::runtime_error(context + " must parse to a GstBin");
  }

  try {
    validateBridgeManagedEndpoints(context, pipeline, expect_bridge_appsrc);
  } catch (...) {
    gst_object_unref(pipeline);
    throw;
  }

  gst_object_unref(pipeline);
}

std::string makeRosValidationPrefix()
{
  std::string prefix = "appsrc name=";
  prefix += kBridgeVideoAppSrcName;
  prefix += " is-live=true block=false format=time do-timestamp=true";
  prefix += " caps=video/x-raw,format=RGB,width=2,height=2,framerate=0/1";
  return prefix;
}

std::string parseVideoTransform(const std::string & raw_transform)
{
  return trim(raw_transform);
}

std::string parseCustomVideoSource(const std::string & entry_id, const std::string & raw_source)
{
  const std::string source = trim(raw_source);
  if (source.empty()) {
    throw std::runtime_error("video custom source '" + entry_id + "' requires a non-empty source");
  }
  return source;
}

void validateVideoTopicRuleTransform(const std::string & entry_id, const std::string & transform)
{
  const std::string context = "video topic rule '" + entry_id + "' transform";
  validatePipelineDescription(context, composeVideoPipelineDescription(makeRosValidationPrefix(), transform), true);
}

void validateCustomVideoSourceFragments(
  const std::string & entry_id, const std::string & source, const std::string & transform)
{
  const std::string context = "video custom source '" + entry_id + "'";
  validatePipelineDescription(context, composeVideoPipelineDescription(source, transform), false);
}

void requireUniqueEntryKey(std::unordered_set<std::string> & seen_keys, const std::string & key, const char * context)
{
  if (!seen_keys.emplace(key).second) {
    throw std::runtime_error(std::string("duplicate ") + context + " '" + key + "'");
  }
}

template <typename EntryMap>
const typename EntryMap::mapped_type & requireUniqueGeneratedEntry(
  std::unordered_set<std::string> & seen_ids,
  const std::string & entry_id,
  const EntryMap & ids_map,
  const char * duplicate_context,
  const char * missing_context)
{
  requireUniqueEntryKey(seen_ids, entry_id, duplicate_context);

  const auto entry_it = ids_map.find(entry_id);
  if (entry_it == ids_map.end()) {
    throw std::runtime_error(std::string(missing_context) + " '" + entry_id + "' is missing generated parameters");
  }

  return entry_it->second;
}

}  // namespace

VideoConfig loadVideoConfig(const Params & params)
{
  VideoConfig config = makeDefaultVideoConfig();
  config.publish = parseVideoPublishConfig(params);
  for (auto & rule : config.ros_topic_rules) {
    rule.publish = config.publish;
  }

  // Rebuild user rules ahead of the built-in catch-all so longest-match selection
  // still works and same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_topic_rule_ids;
  std::unordered_set<std::string> seen_custom_source_ids;
  std::unordered_set<std::string> seen_external_names;
  // Keep video_topic_rule_ids at the root until generate_parameter_library 0.7+
  // is the baseline across the full distro matrix. GPL 0.6.x does not support
  // moving this cleanly to video.topic_rules.ids for every target we test.
  for (const auto & entry_id : params.video_topic_rule_ids) {
    const auto & entry = requireUniqueGeneratedEntry(
      seen_topic_rule_ids,
      entry_id,
      params.video.topic_rules.video_topic_rule_ids_map,
      "video topic rule id",
      "video topic rule");

    const std::string pattern = normalizeVideoRulePattern(entry.pattern);
    const std::string transform = parseVideoTransform(entry.transform);
    validateVideoTopicRuleTransform(entry_id, transform);

    RosTopicRule rule;
    rule.pattern = pattern;
    rule.id = entry_id;
    rule.transform = transform;
    rule.publish =
      mergeVideoPublishConfig(config.publish, parseVideoPublishOverride(entry, "video topic rule '" + entry_id + "'"));
    config.ros_topic_rules.push_back(std::move(rule));
  }

  // Keep video_custom_source_ids at the root until generate_parameter_library
  // 0.7+ is the baseline across the full distro matrix. GPL 0.6.x does not
  // support moving this cleanly to video.custom_sources.ids for every target we
  // test.
  for (const auto & entry_id : params.video_custom_source_ids) {
    const auto & entry = requireUniqueGeneratedEntry(
      seen_custom_source_ids,
      entry_id,
      params.video.custom_sources.video_custom_source_ids_map,
      "video custom source id",
      "video custom source");

    const std::string source_fragment = parseCustomVideoSource(entry_id, entry.source);
    const std::string transform = parseVideoTransform(entry.transform);
    validateCustomVideoSourceFragments(entry_id, source_fragment, transform);

    // Configured sources are keyed by the canonical normalized external name, so
    // spelling variants collapse to one lookup key and one shared stream contract.
    const std::string normalized_external_name = normalizeExternalName(entry_id);
    if (normalized_external_name.empty()) {
      throw std::runtime_error("video custom source '" + entry_id + "' must normalize to a valid external name");
    }
    requireUniqueEntryKey(seen_external_names, normalized_external_name, "configured video external name");

    ConfiguredExternalSource source;
    source.source = source_fragment;
    source.transform = transform;
    source.publish = mergeVideoPublishConfig(
      config.publish, parseVideoPublishOverride(entry, "video custom source '" + entry_id + "'"));
    config.external_sources.emplace(normalized_external_name, std::move(source));
  }

  // Append built-in catch-all after user entries.
  config.ros_topic_rules.insert(
    config.ros_topic_rules.end(),
    std::make_move_iterator(builtin_rules.begin()),
    std::make_move_iterator(builtin_rules.end()));

  return config;
}

namespace
{

AccessPolicy loadAccessPolicy(const Params & params)
{
  AccessPolicyConfig config;
  config.publish.allow = params.access.rules.publish.allow;
  config.publish.deny = params.access.rules.publish.deny;
  config.subscribe.allow = params.access.rules.subscribe.allow;
  config.subscribe.deny = params.access.rules.subscribe.deny;
  config.service.allow = params.access.rules.service.allow;
  config.service.deny = params.access.rules.service.deny;
  return AccessPolicy(config);
}

SubscriptionQosConfig loadSubscriptionQosConfig(const Params & params)
{
  SubscriptionQosConfig config;

  std::unordered_set<std::string> seen_override_ids;
  for (const auto & override_id : params.subscription_qos_override_ids) {
    const auto & entry = requireUniqueGeneratedEntry(
      seen_override_ids,
      override_id,
      params.subscribe.qos_overrides.subscription_qos_override_ids_map,
      "subscription QoS override id",
      "subscription QoS override entry");

    TopicSubscriptionQosOverride override_entry;
    override_entry.id = override_id;
    override_entry.pattern = normalizeRosTopicPattern(entry.pattern, "subscribe.qos_overrides");
    override_entry.reliability = parseSubscriptionQosReliabilityMode(entry.reliability);
    override_entry.durability = parseSubscriptionQosDurabilityMode(entry.durability);
    config.topic_overrides.push_back(std::move(override_entry));
  }

  return config;
}

}  // namespace

RuntimeConfig loadRuntimeConfig(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters_interface,
  const std::string & node_name)
{
  std::string room = kUnsetLogValue;
  std::string identity = kUnsetLogValue;
  const char * auth_mode_name = "unknown";

  try {
    if (parameters_interface == nullptr) {
      throw std::invalid_argument("parameters_interface is required");
    }

    ParamListener param_listener(parameters_interface);
    RuntimeConfig runtime_config;
    runtime_config.loaded_params = param_listener.get_params();
    room =
      runtime_config.loaded_params.livekit.room.empty() ? kUnsetLogValue : runtime_config.loaded_params.livekit.room;

    const LiveKitAuthMode auth_mode = classifyLiveKitAuthMode(runtime_config.loaded_params);
    auth_mode_name = authModeString(auth_mode);
    identity = runtime_config.loaded_params.livekit.identity.empty() ? deriveDefaultIdentity(node_name)
                                                                     : runtime_config.loaded_params.livekit.identity;

    validateLiveKitApiCredentialPair(auth_mode);
    validateTokenTtl(runtime_config.loaded_params, auth_mode);
    runtime_config.connect_config = loadConnectConfig(runtime_config.loaded_params, node_name);
    runtime_config.token_source = loadTokenSource(runtime_config.loaded_params, auth_mode);
    runtime_config.access_policy = loadAccessPolicy(runtime_config.loaded_params);
    runtime_config.subscription_qos_config = loadSubscriptionQosConfig(runtime_config.loaded_params);
    runtime_config.video_config = loadVideoConfig(runtime_config.loaded_params);

    LogEvent(kRuntimeConfigLogger, "runtime_config_loaded")
      .kv("phase", "startup")
      .kvOr("room", runtime_config.connect_config.room, kUnsetLogValue)
      .kvOr("identity", runtime_config.connect_config.identity, kUnsetLogValue)
      .kv("auth_mode", auth_mode_name)
      .info();

    return runtime_config;
  } catch (const std::exception & exc) {
    LogEvent(kRuntimeConfigLogger, "runtime_config_load_failed")
      .kv("phase", "startup")
      .kv("reason", "config_validation_failed")
      .kv("room", room)
      .kv("identity", identity)
      .kv("auth_mode", auth_mode_name)
      .kv("error", exc.what())
      .error();
    throw;
  } catch (...) {
    LogEvent(kRuntimeConfigLogger, "runtime_config_load_failed")
      .kv("phase", "startup")
      .kv("reason", "config_validation_failed")
      .kv("room", room)
      .kv("identity", identity)
      .kv("auth_mode", auth_mode_name)
      .kv("error", "unknown_exception")
      .error();
    throw;
  }
}

}  // namespace livekit_ros2_bridge
