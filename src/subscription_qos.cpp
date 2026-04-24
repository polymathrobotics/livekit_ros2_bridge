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

#include <algorithm>
#include <initializer_list>
#include <optional>
#include <stdexcept>

#include "rmw/qos_string_conversions.h"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename Policy>
struct PublisherPolicySummary
{
  std::optional<Policy> resolved_policy;
  bool mixed = false;
};

template <typename Policy, typename Mutator>
bool qosPoliciesAreCompatible(Policy publisher_policy, Policy subscription_policy, Mutator mutator)
{
  rclcpp::QoS publisher_qos{rclcpp::KeepLast(1)};
  rclcpp::QoS subscription_qos{rclcpp::KeepLast(1)};
  mutator(publisher_qos, publisher_policy);
  mutator(subscription_qos, subscription_policy);
  return rclcpp::qos_check_compatible(publisher_qos, subscription_qos).compatibility == rclcpp::QoSCompatibility::Ok;
}

template <typename Policy>
bool containsPolicy(std::initializer_list<Policy> policies, Policy policy)
{
  return std::find(policies.begin(), policies.end(), policy) != policies.end();
}

template <typename Policy, typename Accessor, typename Mutator>
PublisherPolicySummary<Policy> summarizePublisherPolicy(
  const std::vector<rclcpp::QoS> & publisher_qos_profiles,
  Accessor accessor,
  Mutator mutator,
  std::initializer_list<Policy> candidate_subscription_policies)
{
  PublisherPolicySummary<Policy> summary;
  for (const auto & publisher_qos : publisher_qos_profiles) {
    const Policy policy = accessor(publisher_qos);
    if (!containsPolicy(candidate_subscription_policies, policy)) {
      continue;
    }
    if (!summary.resolved_policy.has_value()) {
      summary.resolved_policy = policy;
      continue;
    }
    if (*summary.resolved_policy != policy) {
      summary.mixed = true;
    }
  }

  if (!summary.resolved_policy.has_value()) {
    return summary;
  }

  std::optional<Policy> compatible_policy;
  for (const Policy subscription_policy : candidate_subscription_policies) {
    bool compatible_with_all_publishers = true;
    for (const auto & publisher_qos : publisher_qos_profiles) {
      const Policy publisher_policy = accessor(publisher_qos);
      if (!containsPolicy(candidate_subscription_policies, publisher_policy)) {
        continue;
      }
      if (!qosPoliciesAreCompatible(publisher_policy, subscription_policy, mutator)) {
        compatible_with_all_publishers = false;
        break;
      }
    }
    if (compatible_with_all_publishers) {
      compatible_policy = subscription_policy;
    }
  }

  if (compatible_policy.has_value()) {
    summary.resolved_policy = *compatible_policy;
  } else if (summary.mixed) {
    summary.resolved_policy = *(candidate_subscription_policies.end() - 1);
  }

  return summary;
}

}  // namespace

ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<rclcpp::QoS> & publisher_qos_profiles)
{
  ResolvedSubscriptionQos resolved;
  resolved.qos = base_qos;
  resolved.publisher_count = publisher_qos_profiles.size();

  const TopicSubscriptionQosOverride * match = nullptr;
  if (config != nullptr) {
    // Prefer the most specific matching pattern. Equal-length matches keep the
    // first configured override so configuration order remains the tiebreaker.
    for (const auto & candidate : config->topic_overrides) {
      if (!rosResourceMatchesPattern(topic, candidate.pattern)) {
        continue;
      }
      if (match == nullptr || candidate.pattern.size() > match->pattern.size()) {
        match = &candidate;
      }
    }
  }

  if (match != nullptr) {
    resolved.override_id = match->id;
    resolved.override_pattern = match->pattern;
  }

  // Only explicit publisher policies participate in resolution. Unknown and
  // system-default entries do not provide a concrete policy to inherit.
  const auto reliability = summarizePublisherPolicy(
    publisher_qos_profiles,
    [](const rclcpp::QoS & publisher_qos) { return publisher_qos.reliability(); },
    [](rclcpp::QoS & qos, rclcpp::ReliabilityPolicy policy) { qos.reliability(policy); },
    {rclcpp::ReliabilityPolicy::Reliable, rclcpp::ReliabilityPolicy::BestEffort});
  const auto durability = summarizePublisherPolicy(
    publisher_qos_profiles,
    [](const rclcpp::QoS & publisher_qos) { return publisher_qos.durability(); },
    [](rclcpp::QoS & qos, rclcpp::DurabilityPolicy policy) { qos.durability(policy); },
    {rclcpp::DurabilityPolicy::TransientLocal, rclcpp::DurabilityPolicy::Volatile});
  resolved.mixed_reliability = reliability.mixed;
  resolved.mixed_durability = durability.mixed;

  if (match == nullptr && !reliability.resolved_policy.has_value() && !durability.resolved_policy.has_value()) {
    // No override matched and publishers exposed no concrete policy, so the
    // caller's base QoS is already the final answer.
    return resolved;
  }

  // Each QoS axis resolves independently. A non-auto override wins for that
  // axis; otherwise we consume publisher QoS only when it exposes a concrete
  // policy for that same axis. When publishers disagree, prefer the weaker
  // compatible policy so one subscription can attach to every known publisher.
  if (match != nullptr && match->reliability.has_value()) {
    resolved.qos.reliability(*match->reliability);
  } else if (reliability.resolved_policy.has_value()) {
    resolved.qos.reliability(*reliability.resolved_policy);
    resolved.used_publisher_qos = true;
  }

  if (match != nullptr && match->durability.has_value()) {
    resolved.qos.durability(*match->durability);
  } else if (durability.resolved_policy.has_value()) {
    resolved.qos.durability(*durability.resolved_policy);
    resolved.used_publisher_qos = true;
  }

  if (match != nullptr) {
    resolved.source = SubscriptionQosResolutionSource::Override;
  } else if (resolved.used_publisher_qos) {
    resolved.source = SubscriptionQosResolutionSource::PublisherQos;
  }

  return resolved;
}

ResolvedSubscriptionQos resolveSubscriptionQos(
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & graph,
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config)
{
  std::vector<rclcpp::QoS> publisher_qos_profiles;
  if (graph == nullptr) {
    throw std::invalid_argument("subscription QoS graph interface is null");
  }
  // Resolve against a single graph snapshot. The publisher set may change
  // immediately after this query, but we keep one consistent view per call.
  const auto publisher_infos = graph->get_publishers_info_by_topic(std::string(topic));
  publisher_qos_profiles.reserve(publisher_infos.size());
  for (const auto & info : publisher_infos) {
    publisher_qos_profiles.push_back(info.qos_profile());
  }
  return resolveSubscriptionQos(topic, base_qos, config, publisher_qos_profiles);
}

const char * subscriptionQosSourceString(SubscriptionQosResolutionSource source)
{
  switch (source) {
    case SubscriptionQosResolutionSource::Fallback:
      return "fallback";
    case SubscriptionQosResolutionSource::PublisherQos:
      return "publisher_qos";
    case SubscriptionQosResolutionSource::Override:
      return "override";
  }

  return "unknown";
}

const char * subscriptionQosReliabilityString(rclcpp::ReliabilityPolicy policy)
{
  const char * name = rmw_qos_reliability_policy_to_str(static_cast<rmw_qos_reliability_policy_t>(policy));
  return name == nullptr ? "unknown" : name;
}

const char * subscriptionQosDurabilityString(rclcpp::DurabilityPolicy policy)
{
  const char * name = rmw_qos_durability_policy_to_str(static_cast<rmw_qos_durability_policy_t>(policy));
  return name == nullptr ? "unknown" : name;
}

}  // namespace livekit_ros2_bridge
