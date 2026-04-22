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

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"

namespace livekit_ros2_bridge
{

struct TopicSubscriptionQosOverride
{
  std::string id;
  // Normalized topic pattern. If multiple overrides match, the longest pattern
  // wins; equal-length ties preserve config order.
  std::string pattern;
  std::optional<rclcpp::ReliabilityPolicy> reliability;
  std::optional<rclcpp::DurabilityPolicy> durability;
};

struct SubscriptionQosConfig
{
  std::vector<TopicSubscriptionQosOverride> topic_overrides;
};

struct PublisherQos
{
  // Pure resolver input distilled from the ROS graph. `Unknown` and
  // `SystemDefault` mean "no concrete policy observed" for that axis.
  rclcpp::ReliabilityPolicy reliability = rclcpp::ReliabilityPolicy::Unknown;
  rclcpp::DurabilityPolicy durability = rclcpp::DurabilityPolicy::Unknown;
};

enum class SubscriptionQosResolutionSource
{
  kFallback,
  kPublisherQos,
  kOverride
};

struct ResolvedSubscriptionQos
{
  // Starts from the caller's base QoS. Resolution only mutates reliability and
  // durability; all other QoS axes stay as provided by the caller.
  rclcpp::QoS qos{rclcpp::KeepLast(2)};
  // Highest-precedence contributor to the final result. This remains
  // `kOverride` when a matching override leaves one axis on `kAuto` and
  // publisher QoS fills that axis.
  SubscriptionQosResolutionSource source = SubscriptionQosResolutionSource::kFallback;
  // True when at least one resolved axis came from discovered publisher QoS.
  bool used_publisher_qos = false;
  // Diagnostic flags derived from the discovered publisher set before override
  // application. They explain why auto resolution chose the weaker policy.
  bool mixed_reliability = false;
  bool mixed_durability = false;
  // Number of publisher profiles considered during resolution.
  std::size_t publisher_count = 0;
  // Populated whenever an override matches this topic, even if some axes stay
  // on `kAuto` and are filled from publisher QoS or the caller's base QoS.
  std::string override_id;
  std::string override_pattern;
};

// Resolve per-topic subscription QoS with per-axis precedence:
// matching override -> discovered publisher QoS -> base_qos. `config` may be
// null. `publishers` should describe one logical snapshot of the topic's
// publishers so the result is internally consistent.
ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<PublisherQos> & publishers);

// Convenience overload that snapshots publisher QoS from the ROS graph before
// applying the same precedence rules as the vector-based overload.
ResolvedSubscriptionQos resolveSubscriptionQos(
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & graph,
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config);

const char * subscriptionQosSourceString(SubscriptionQosResolutionSource source);
const char * subscriptionQosReliabilityString(rclcpp::ReliabilityPolicy policy);
const char * subscriptionQosDurabilityString(rclcpp::DurabilityPolicy policy);

}  // namespace livekit_ros2_bridge
