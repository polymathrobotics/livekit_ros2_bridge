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

#include <unistd.h>

#include <chrono>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRuntimeConfigLogger = rclcpp::get_logger("livekit_ros2_bridge.runtime_config");

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
      "livekit.token_ttl_seconds must be > 0 when livekit.api_key/livekit.api_secret mint bridge or "
      "video sidecar tokens");
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

std::string normalizeVideoRulePattern(std::string_view raw_pattern)
{
  const std::string trimmed = trim(raw_pattern);
  if (trimmed.empty()) {
    throw std::runtime_error("video topic rule pattern must not be empty");
  }
  if (trimmed == "*") {
    return "/*";
  }
  if (trimmed.size() >= 2 && trimmed.substr(trimmed.size() - 2) == "/*") {
    const std::string normalized = normalizeRosResourceName(trimmed.substr(0, trimmed.size() - 2));
    if (normalized.empty()) {
      throw std::runtime_error("video topic rule pattern must normalize to a valid ROS resource");
    }
    return normalized + "/*";
  }

  const std::string normalized = normalizeRosResourceName(trimmed);
  if (normalized.empty()) {
    throw std::runtime_error("video topic rule pattern must normalize to a valid ROS resource");
  }
  return normalized;
}

bool isSupportedRosPipelineAlias(std::string_view alias)
{
  return alias == kImagePipelineAlias || alias == kCompressedImagePipelineAlias || alias == kDefaultPipelineAlias;
}

bool isSupportedConfiguredSourcePipelineAlias(std::string_view alias)
{
  return alias == kDefaultPipelineAlias;
}

PipelineMap parsePipelineEntries(const std::string & entry_id, const std::vector<std::string> & raw_entries)
{
  PipelineMap pipelines;
  for (const auto & raw : raw_entries) {
    const auto eq_pos = raw.find('=');
    if (eq_pos == std::string::npos || eq_pos == 0) {
      throw std::runtime_error(
        "video entry '" + entry_id + "' has malformed pipeline entry (expected alias=pipeline): '" + raw + "'");
    }

    const std::string alias = trim(raw.substr(0, eq_pos));
    if (alias.empty()) {
      throw std::runtime_error(
        "video entry '" + entry_id + "' has malformed pipeline entry (expected alias=pipeline): '" + raw + "'");
    }

    const std::string pipeline = trim(raw.substr(eq_pos + 1));
    if (pipeline.empty()) {
      throw std::runtime_error("video entry '" + entry_id + "' has empty pipeline for alias '" + alias + "'");
    }

    const auto inserted = pipelines.emplace(alias, pipeline).second;
    if (!inserted) {
      throw std::runtime_error("video entry '" + entry_id + "' has duplicate pipeline alias '" + alias + "'");
    }
  }

  return pipelines;
}

void validateRosPipelines(const std::string & entry_id, const PipelineMap & pipelines)
{
  if (pipelines.empty()) {
    throw std::runtime_error(
      "video entry '" + entry_id +
      "' (ros kind) requires at least one pipeline alias from "
      "[image, compressed_image, default]");
  }

  for (const auto & pipeline_entry : pipelines) {
    if (!isSupportedRosPipelineAlias(pipeline_entry.first)) {
      throw std::runtime_error(
        "video entry '" + entry_id + "' (ros kind) has unsupported pipeline alias '" + pipeline_entry.first + "'");
    }
  }
}

void validateConfiguredSourcePipelines(const std::string & entry_id, const PipelineMap & pipelines)
{
  if (pipelines.empty()) {
    throw std::runtime_error("video entry '" + entry_id + "' (pipeline kind) requires a 'default' pipeline key");
  }

  for (const auto & pipeline_entry : pipelines) {
    if (!isSupportedConfiguredSourcePipelineAlias(pipeline_entry.first)) {
      throw std::runtime_error(
        "video entry '" + entry_id + "' (pipeline kind) has unsupported pipeline alias '" + pipeline_entry.first + "'");
    }
  }

  if (pipelines.find(kDefaultPipelineAlias) == pipelines.end()) {
    throw std::runtime_error("video entry '" + entry_id + "' (pipeline kind) requires a 'default' pipeline key");
  }
}

void requireUniqueEntryKey(std::unordered_set<std::string> & seen_keys, const std::string & key, const char * context)
{
  if (!seen_keys.emplace(key).second) {
    throw std::runtime_error(std::string("duplicate ") + context + " '" + key + "'");
  }
}

template <typename EntryMap>
const typename EntryMap::mapped_type & requireUniqueGeneratedVideoEntry(
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

  // Rebuild user rules ahead of the built-in catch-all so longest-match selection
  // still works and same-length ties stay first-declared.
  auto builtin_rules = std::move(config.ros_topic_rules);
  config.ros_topic_rules.clear();

  std::unordered_set<std::string> seen_entry_ids;
  std::unordered_set<std::string> seen_external_names;
  for (const auto & entry_id : params.videos.ids) {
    const auto & entry = requireUniqueGeneratedVideoEntry(
      seen_entry_ids, entry_id, params.videos.ids_map, "video entry id", "video entry");

    PipelineMap pipelines = parsePipelineEntries(entry_id, entry.pipelines);

    if (entry.kind == "ros") {
      validateRosPipelines(entry_id, pipelines);

      const std::string pattern = normalizeVideoRulePattern(entry.pattern);

      RosTopicRule rule;
      rule.pattern = pattern;
      rule.id = entry_id;
      rule.pipelines = std::move(pipelines);
      config.ros_topic_rules.push_back(std::move(rule));
    } else if (entry.kind == "pipeline") {
      validateConfiguredSourcePipelines(entry_id, pipelines);

      // Configured sources are keyed by the canonical normalized external name,
      // so spelling variants collapse to one lookup key and one sidecar contract.
      const std::string normalized_external_name = normalizeExternalName(entry_id);
      if (normalized_external_name.empty()) {
        throw std::runtime_error("video entry '" + entry_id + "' must normalize to a valid external name");
      }
      requireUniqueEntryKey(seen_external_names, normalized_external_name, "configured video external name");

      ConfiguredPipelineSource source;
      source.pipeline = pipelines.at(kDefaultPipelineAlias);
      config.pipeline_sources.emplace(normalized_external_name, std::move(source));
    } else {
      throw std::runtime_error("video entry '" + entry_id + "' has unsupported kind '" + entry.kind + "'");
    }
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

std::optional<VideoSidecarSupervisor::Config> loadVideoSidecarConfig(
  const Params & params, const RoomConnectionConfig & connect_config, const LiveKitAuthMode auth_mode)
{
  validateLiveKitApiCredentialPair(auth_mode);

  // Sidecars need API credentials whenever they mint their own publisher token.
  // A static bridge token can coexist with that, but without minting there is no sidecar config to build.
  if (!enablesApiMintedTokens(auth_mode)) {
    return std::nullopt;
  }

  VideoSidecarSupervisor::Config config;
  config.livekit_url = connect_config.url;
  config.livekit_room = connect_config.room;
  config.api_key = params.livekit.api_key;
  config.api_secret = params.livekit.api_secret;
  config.token_ttl = std::chrono::seconds(params.livekit.token_ttl_seconds);
  config.token_refresh_margin = std::chrono::seconds(params.livekit.token_refresh_margin_seconds);
  config.bridge_identity = connect_config.identity;
  return config;
}

}  // namespace

RuntimeConfig loadRuntimeConfig(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters_interface,
  const std::string & node_name)
{
  std::string room = "<unset>";
  std::string identity = "<unset>";
  const char * auth_mode_name = "unknown";
  bool sidecar_enabled = false;

  try {
    if (parameters_interface == nullptr) {
      throw std::invalid_argument("parameters_interface is required");
    }

    ParamListener param_listener(parameters_interface);
    RuntimeConfig runtime_config;
    runtime_config.loaded_params = param_listener.get_params();
    room = runtime_config.loaded_params.livekit.room.empty() ? "<unset>" : runtime_config.loaded_params.livekit.room;

    const LiveKitAuthMode auth_mode = classifyLiveKitAuthMode(runtime_config.loaded_params);
    auth_mode_name = authModeString(auth_mode);
    identity = runtime_config.loaded_params.livekit.identity.empty() ? deriveDefaultIdentity(node_name)
                                                                     : runtime_config.loaded_params.livekit.identity;

    validateLiveKitApiCredentialPair(auth_mode);
    validateTokenTtl(runtime_config.loaded_params, auth_mode);
    runtime_config.connect_config = loadConnectConfig(runtime_config.loaded_params, node_name);
    runtime_config.token_source = loadTokenSource(runtime_config.loaded_params, auth_mode);
    runtime_config.access_policy = loadAccessPolicy(runtime_config.loaded_params);
    runtime_config.video_config = loadVideoConfig(runtime_config.loaded_params);
    runtime_config.video_sidecar_config =
      loadVideoSidecarConfig(runtime_config.loaded_params, runtime_config.connect_config, auth_mode);
    sidecar_enabled = runtime_config.video_sidecar_config.has_value();

    LogEvent(kRuntimeConfigLogger, "runtime_config_loaded")
      .kv("phase", "startup")
      .kvOr("room", runtime_config.connect_config.room, "<unset>")
      .kvOr("identity", runtime_config.connect_config.identity, "<unset>")
      .kv("auth_mode", auth_mode_name)
      .kv("sidecar_enabled", sidecar_enabled)
      .info();

    return runtime_config;
  } catch (const std::exception & exc) {
    LogEvent(kRuntimeConfigLogger, "runtime_config_load_failed")
      .kv("phase", "startup")
      .kv("reason", "config_validation_failed")
      .kv("room", room)
      .kv("identity", identity)
      .kv("auth_mode", auth_mode_name)
      .kv("sidecar_enabled", sidecar_enabled)
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
      .kv("sidecar_enabled", sidecar_enabled)
      .kv("error", "unknown_exception")
      .error();
    throw;
  }
}

}  // namespace livekit_ros2_bridge
