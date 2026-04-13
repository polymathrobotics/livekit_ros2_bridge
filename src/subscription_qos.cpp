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

#include "rclcpp/version.h"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<PublisherQosProfile> & publisher_profiles)
{
  ResolvedSubscriptionQos resolved;
  resolved.qos = base_qos;
  resolved.publisher_count = publisher_profiles.size();

  const TopicSubscriptionQosOverride * override_match = nullptr;
  if (config != nullptr) {
    // Prefer the most specific matching pattern. Equal-length matches keep the
    // first configured override so configuration order remains the tiebreaker.
    for (const auto & candidate : config->topic_overrides) {
      if (!rosResourceMatchesPattern(topic, candidate.pattern)) {
        continue;
      }
      if (override_match == nullptr || candidate.pattern.size() > override_match->pattern.size()) {
        override_match = &candidate;
      }
    }
  }

  if (override_match != nullptr) {
    resolved.override_id = override_match->id;
    resolved.override_pattern = override_match->pattern;
  }

  // Only explicit publisher policies participate in resolution. Unknown and
  // system-default entries do not provide a concrete policy to inherit.
  bool has_reliable = false;
  bool has_best_effort = false;
  bool has_volatile = false;
  bool has_transient_local = false;
  for (const auto & profile : publisher_profiles) {
    switch (profile.reliability) {
      case rclcpp::ReliabilityPolicy::Reliable:
        has_reliable = true;
        break;
      case rclcpp::ReliabilityPolicy::BestEffort:
        has_best_effort = true;
        break;
      default:
        break;
    }

    switch (profile.durability) {
      case rclcpp::DurabilityPolicy::Volatile:
        has_volatile = true;
        break;
      case rclcpp::DurabilityPolicy::TransientLocal:
        has_transient_local = true;
        break;
      default:
        break;
    }
  }
  resolved.mixed_reliability = has_reliable && has_best_effort;
  resolved.mixed_durability = has_volatile && has_transient_local;

  const bool has_concrete_publisher_qos = has_best_effort || has_reliable || has_volatile || has_transient_local;
  if (override_match == nullptr && !has_concrete_publisher_qos) {
    // No override matched and publishers exposed no concrete policy, so the
    // caller's base QoS is already the final answer.
    return resolved;
  }

  // Each QoS axis resolves independently. A non-auto override wins for that
  // axis; otherwise we consume publisher QoS only when it exposes a concrete
  // policy for that same axis. When publishers disagree, prefer the weaker
  // compatible policy so one subscription can attach to every known publisher.
  if (override_match != nullptr && override_match->reliability != SubscriptionQosReliabilityMode::kAuto) {
    resolved.qos.reliability(
      override_match->reliability == SubscriptionQosReliabilityMode::kBestEffort ? rclcpp::ReliabilityPolicy::BestEffort
                                                                                 : rclcpp::ReliabilityPolicy::Reliable);
  } else if (has_best_effort || has_reliable) {
    resolved.qos.reliability(
      has_best_effort ? rclcpp::ReliabilityPolicy::BestEffort : rclcpp::ReliabilityPolicy::Reliable);
    resolved.used_publisher_qos = true;
  }

  if (override_match != nullptr && override_match->durability != SubscriptionQosDurabilityMode::kAuto) {
    resolved.qos.durability(
      override_match->durability == SubscriptionQosDurabilityMode::kTransientLocal
        ? rclcpp::DurabilityPolicy::TransientLocal
        : rclcpp::DurabilityPolicy::Volatile);
  } else if (has_volatile || has_transient_local) {
    resolved.qos.durability(
      has_volatile ? rclcpp::DurabilityPolicy::Volatile : rclcpp::DurabilityPolicy::TransientLocal);
    resolved.used_publisher_qos = true;
  }

  if (override_match != nullptr) {
    resolved.source = SubscriptionQosResolutionSource::kOverride;
  } else if (resolved.used_publisher_qos) {
    resolved.source = SubscriptionQosResolutionSource::kPublisherQos;
  }

  return resolved;
}

ResolvedSubscriptionQos resolveSubscriptionQos(
  const rclcpp::Node & node, std::string_view topic, const rclcpp::QoS & base_qos, const SubscriptionQosConfig * config)
{
  std::vector<PublisherQosProfile> publisher_profiles;
  // Resolve against a single graph snapshot. The publisher set may change
  // immediately after this query, but we keep one consistent view per call.
  const auto publishers = node.get_publishers_info_by_topic(std::string(topic));
  publisher_profiles.reserve(publishers.size());
  for (const auto & publisher : publishers) {
    const auto & publisher_qos = publisher.qos_profile();
    publisher_profiles.push_back({publisher_qos.reliability(), publisher_qos.durability()});
  }
  return resolveSubscriptionQos(topic, base_qos, config, publisher_profiles);
}

const char * subscriptionQosSourceString(SubscriptionQosResolutionSource source)
{
  switch (source) {
    case SubscriptionQosResolutionSource::kFallback:
      return "fallback";
    case SubscriptionQosResolutionSource::kPublisherQos:
      return "publisher_qos";
    case SubscriptionQosResolutionSource::kOverride:
      return "override";
  }

  return "unknown";
}

const char * subscriptionQosReliabilityString(rclcpp::ReliabilityPolicy policy)
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

const char * subscriptionQosDurabilityString(rclcpp::DurabilityPolicy policy)
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
