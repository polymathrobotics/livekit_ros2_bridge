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
#include <cstdlib>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/qos_string_conversions.h"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"
#include "video/stream_config.hpp"

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
    config.video_stream = video::loadConfig(params);

    return config;
  } catch (...) {
    LogEvent(kLogger, "node_config_load_failed")
      .field("stage", stage)
      .fieldException("error", std::current_exception())
      .error();

    throw;
  }
}

}  // namespace livekit_ros2_bridge
