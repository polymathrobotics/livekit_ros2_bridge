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

#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename Policy>
struct PolicySummary
{
  std::optional<Policy> policy;
  bool mixed = false;
};

template <typename Policy, typename Mutator>
bool areCompatible(Policy offered, Policy requested, Mutator mutator)
{
  rclcpp::QoS publisher{rclcpp::KeepLast(1)};
  rclcpp::QoS subscription{rclcpp::KeepLast(1)};
  mutator(publisher, offered);
  mutator(subscription, requested);
  return rclcpp::qos_check_compatible(publisher, subscription).compatibility == rclcpp::QoSCompatibility::Ok;
}

template <typename Policy>
bool contains(std::initializer_list<Policy> policies, Policy policy)
{
  return std::find(policies.begin(), policies.end(), policy) != policies.end();
}

template <typename Policy, typename Accessor, typename Mutator>
PolicySummary<Policy> summarizePolicy(
  const std::vector<rclcpp::QoS> & publisher_qos,
  Accessor accessor,
  Mutator mutator,
  std::initializer_list<Policy> candidates)
{
  PolicySummary<Policy> summary;
  for (const auto & qos : publisher_qos) {
    const Policy policy = accessor(qos);
    if (!contains(candidates, policy)) {
      // Unknown and system-default entries are not concrete publisher offers.
      continue;
    }
    if (!summary.policy.has_value()) {
      summary.policy = policy;
      continue;
    }
    if (*summary.policy != policy) {
      summary.mixed = true;
    }
  }

  if (!summary.policy.has_value()) {
    return summary;
  }

  // Candidates run strongest to weakest; keep the weakest compatible request.
  std::optional<Policy> compatible;
  for (const Policy requested : candidates) {
    bool all_compatible = true;
    for (const auto & qos : publisher_qos) {
      const Policy offered = accessor(qos);
      if (!contains(candidates, offered)) {
        continue;
      }
      if (!areCompatible(offered, requested, mutator)) {
        all_compatible = false;
        break;
      }
    }
    if (all_compatible) {
      compatible = requested;
    }
  }

  if (compatible.has_value()) {
    summary.policy = *compatible;
  } else if (summary.mixed) {
    // If compatibility cannot prove a common request across mixed publishers,
    // prefer the least restrictive candidate over the first publisher seen.
    summary.policy = *(candidates.end() - 1);
  }

  return summary;
}

}  // namespace

ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base,
  const SubscriptionQosConfig * config,
  const std::vector<rclcpp::QoS> & publisher_qos)
{
  ResolvedSubscriptionQos resolved;
  resolved.qos = base;
  resolved.publisher_count = publisher_qos.size();

  const TopicSubscriptionQosOverride * match =
    config == nullptr
      ? nullptr
      : findBestRosResourcePatternMatch(config->topic_overrides, topic, &TopicSubscriptionQosOverride::pattern);

  if (match != nullptr) {
    resolved.override_id = match->id;
  }

  const auto reliability = summarizePolicy(
    publisher_qos,
    [](const rclcpp::QoS & qos) { return qos.reliability(); },
    [](rclcpp::QoS & qos, rclcpp::ReliabilityPolicy policy) { qos.reliability(policy); },
    {rclcpp::ReliabilityPolicy::Reliable, rclcpp::ReliabilityPolicy::BestEffort});
  const auto durability = summarizePolicy(
    publisher_qos,
    [](const rclcpp::QoS & qos) { return qos.durability(); },
    [](rclcpp::QoS & qos, rclcpp::DurabilityPolicy policy) { qos.durability(policy); },
    {rclcpp::DurabilityPolicy::TransientLocal, rclcpp::DurabilityPolicy::Volatile});
  resolved.mixed_reliability = reliability.mixed;
  resolved.mixed_durability = durability.mixed;

  if (match == nullptr && !reliability.policy.has_value() && !durability.policy.has_value()) {
    return resolved;
  }

  bool used_publisher_policy = false;
  if (match != nullptr && match->reliability.has_value()) {
    resolved.qos.reliability(*match->reliability);
  } else if (reliability.policy.has_value()) {
    resolved.qos.reliability(*reliability.policy);
    used_publisher_policy = true;
  }

  if (match != nullptr && match->durability.has_value()) {
    resolved.qos.durability(*match->durability);
  } else if (durability.policy.has_value()) {
    resolved.qos.durability(*durability.policy);
    used_publisher_policy = true;
  }

  if (match != nullptr) {
    resolved.source = SubscriptionQosResolutionSource::Override;
  } else if (used_publisher_policy) {
    resolved.source = SubscriptionQosResolutionSource::PublisherQos;
  }

  return resolved;
}

ResolvedSubscriptionQos resolveSubscriptionQos(
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & graph,
  std::string_view topic,
  const rclcpp::QoS & base,
  const SubscriptionQosConfig * config)
{
  std::vector<rclcpp::QoS> publisher_qos;
  if (graph == nullptr) {
    throw std::invalid_argument("subscription QoS graph interface is null");
  }
  // Keep one graph view even if publishers change after this query.
  const auto publishers = graph->get_publishers_info_by_topic(std::string(topic));
  publisher_qos.reserve(publishers.size());
  for (const auto & publisher : publishers) {
    publisher_qos.push_back(publisher.qos_profile());
  }
  return resolveSubscriptionQos(topic, base, config, publisher_qos);
}

}  // namespace livekit_ros2_bridge
