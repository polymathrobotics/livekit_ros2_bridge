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
#include <stdexcept>

#include "rclcpp/version.h"
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

template <typename Policy, typename Accessor>
PublisherPolicySummary<Policy> summarizePublisherPolicy(
  const std::vector<PublisherQos> & publishers, Accessor accessor, Policy weaker_policy, Policy stronger_policy)
{
  PublisherPolicySummary<Policy> summary;
  for (const auto & publisher : publishers) {
    const Policy policy = accessor(publisher);
    if (policy != weaker_policy && policy != stronger_policy) {
      continue;
    }
    if (!summary.resolved_policy.has_value()) {
      summary.resolved_policy = policy;
      continue;
    }
    if (*summary.resolved_policy != policy) {
      summary.mixed = true;
      summary.resolved_policy = weaker_policy;
    }
  }
  return summary;
}

}  // namespace

ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<PublisherQos> & publishers)
{
  ResolvedSubscriptionQos resolved;
  resolved.qos = base_qos;
  resolved.publisher_count = publishers.size();

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
    publishers,
    [](const PublisherQos & publisher) { return publisher.reliability; },
    rclcpp::ReliabilityPolicy::BestEffort,
    rclcpp::ReliabilityPolicy::Reliable);
  const auto durability = summarizePublisherPolicy(
    publishers,
    [](const PublisherQos & publisher) { return publisher.durability; },
    rclcpp::DurabilityPolicy::Volatile,
    rclcpp::DurabilityPolicy::TransientLocal);
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
  std::vector<PublisherQos> publishers;
  if (graph == nullptr) {
    throw std::invalid_argument("subscription QoS graph interface is null");
  }
  // Resolve against a single graph snapshot. The publisher set may change
  // immediately after this query, but we keep one consistent view per call.
  const auto publisher_infos = graph->get_publishers_info_by_topic(std::string(topic));
  publishers.reserve(publisher_infos.size());
  for (const auto & info : publisher_infos) {
    const auto & qos = info.qos_profile();
    publishers.push_back({qos.reliability(), qos.durability()});
  }
  return resolveSubscriptionQos(topic, base_qos, config, publishers);
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
