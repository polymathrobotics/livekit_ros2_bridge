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

#include "subscription_qos.hpp"

#include <optional>

#include "rclcpp/version.h"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const TopicSubscriptionQosOverride * selectOverride(
  const SubscriptionQosConfig * config, const std::string_view normalized_topic)
{
  if (config == nullptr) {
    return nullptr;
  }

  const TopicSubscriptionQosOverride * best = nullptr;
  for (const auto & candidate : config->topic_overrides) {
    if (!rosResourceMatchesPattern(normalized_topic, candidate.pattern)) {
      continue;
    }
    if (best == nullptr || candidate.pattern.size() > best->pattern.size()) {
      best = &candidate;
    }
  }

  return best;
}

std::optional<rclcpp::ReliabilityPolicy> inferReliability(
  const std::vector<ObservedPublisherQosProfile> & publisher_profiles, bool & mixed_reliability)
{
  bool saw_reliable = false;
  bool saw_best_effort = false;
  for (const auto & profile : publisher_profiles) {
    switch (profile.reliability) {
      case rclcpp::ReliabilityPolicy::Reliable:
        saw_reliable = true;
        break;
      case rclcpp::ReliabilityPolicy::BestEffort:
        saw_best_effort = true;
        break;
      default:
        break;
    }
  }

  mixed_reliability = saw_reliable && saw_best_effort;
  if (saw_best_effort) {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }
  if (saw_reliable) {
    return rclcpp::ReliabilityPolicy::Reliable;
  }
  return std::nullopt;
}

std::optional<rclcpp::DurabilityPolicy> inferDurability(
  const std::vector<ObservedPublisherQosProfile> & publisher_profiles, bool & mixed_durability)
{
  bool saw_volatile = false;
  bool saw_transient_local = false;
  for (const auto & profile : publisher_profiles) {
    switch (profile.durability) {
      case rclcpp::DurabilityPolicy::Volatile:
        saw_volatile = true;
        break;
      case rclcpp::DurabilityPolicy::TransientLocal:
        saw_transient_local = true;
        break;
      default:
        break;
    }
  }

  mixed_durability = saw_volatile && saw_transient_local;
  if (saw_volatile) {
    return rclcpp::DurabilityPolicy::Volatile;
  }
  if (saw_transient_local) {
    return rclcpp::DurabilityPolicy::TransientLocal;
  }
  return std::nullopt;
}

}  // namespace

ResolvedSubscriptionQos resolveTopicSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<ObservedPublisherQosProfile> & publisher_profiles)
{
  ResolvedSubscriptionQos resolved;
  resolved.qos = base_qos;

  const TopicSubscriptionQosOverride * matched_override = selectOverride(config, topic);
  if (matched_override != nullptr) {
    resolved.matched_override_id = matched_override->id;
    resolved.matched_override_pattern = matched_override->pattern;
  }

  bool inferred_reliability_used = false;
  bool inferred_durability_used = false;

  const std::optional<rclcpp::ReliabilityPolicy> inferred_reliability =
    inferReliability(publisher_profiles, resolved.mixed_reliability);
  if (matched_override != nullptr && matched_override->reliability != SubscriptionQosReliabilityMode::kAuto) {
    resolved.qos.reliability(
      matched_override->reliability == SubscriptionQosReliabilityMode::kBestEffort
        ? rclcpp::ReliabilityPolicy::BestEffort
        : rclcpp::ReliabilityPolicy::Reliable);
  } else if (inferred_reliability.has_value()) {
    resolved.qos.reliability(*inferred_reliability);
    inferred_reliability_used = true;
  }

  const std::optional<rclcpp::DurabilityPolicy> inferred_durability =
    inferDurability(publisher_profiles, resolved.mixed_durability);
  if (matched_override != nullptr && matched_override->durability != SubscriptionQosDurabilityMode::kAuto) {
    resolved.qos.durability(
      matched_override->durability == SubscriptionQosDurabilityMode::kTransientLocal
        ? rclcpp::DurabilityPolicy::TransientLocal
        : rclcpp::DurabilityPolicy::Volatile);
  } else if (inferred_durability.has_value()) {
    resolved.qos.durability(*inferred_durability);
    inferred_durability_used = true;
  }

  resolved.used_publisher_info = inferred_reliability_used || inferred_durability_used;
  if (matched_override != nullptr) {
    resolved.source = SubscriptionQosResolutionSource::kOverride;
  } else if (resolved.used_publisher_info) {
    resolved.source = SubscriptionQosResolutionSource::kInspection;
  } else {
    resolved.source = SubscriptionQosResolutionSource::kFallback;
  }

  return resolved;
}

ResolvedSubscriptionQos resolveTopicSubscriptionQos(
  const rclcpp::Node & node, std::string_view topic, const rclcpp::QoS & base_qos, const SubscriptionQosConfig * config)
{
  std::vector<ObservedPublisherQosProfile> publisher_profiles;
  const auto infos = node.get_publishers_info_by_topic(std::string(topic));
  publisher_profiles.reserve(infos.size());
  for (const auto & info : infos) {
    const auto & qos = info.qos_profile();
    publisher_profiles.push_back({qos.reliability(), qos.durability()});
  }
  return resolveTopicSubscriptionQos(topic, base_qos, config, publisher_profiles);
}

const char * subscriptionQosResolutionSourceToString(SubscriptionQosResolutionSource source)
{
  switch (source) {
    case SubscriptionQosResolutionSource::kFallback:
      return "fallback";
    case SubscriptionQosResolutionSource::kInspection:
      return "inspection";
    case SubscriptionQosResolutionSource::kOverride:
      return "override";
  }

  return "unknown";
}

const char * reliabilityPolicyToString(rclcpp::ReliabilityPolicy policy)
{
  switch (policy) {
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
#if RCLCPP_VERSION_GTE(28, 0, 0)
    case rclcpp::ReliabilityPolicy::BestAvailable:
      return "best_available";
#endif
    case rclcpp::ReliabilityPolicy::SystemDefault:
      return "system_default";
    case rclcpp::ReliabilityPolicy::Unknown:
      return "unknown";
  }

  return "unknown";
}

const char * durabilityPolicyToString(rclcpp::DurabilityPolicy policy)
{
  switch (policy) {
    case rclcpp::DurabilityPolicy::Volatile:
      return "volatile";
    case rclcpp::DurabilityPolicy::TransientLocal:
      return "transient_local";
#if RCLCPP_VERSION_GTE(28, 0, 0)
    case rclcpp::DurabilityPolicy::BestAvailable:
      return "best_available";
#endif
    case rclcpp::DurabilityPolicy::SystemDefault:
      return "system_default";
    case rclcpp::DurabilityPolicy::Unknown:
      return "unknown";
  }

  return "unknown";
}

}  // namespace livekit_ros2_bridge
