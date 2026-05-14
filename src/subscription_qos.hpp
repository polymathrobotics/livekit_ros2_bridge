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
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

struct TopicSubscriptionQosOverride
{
  std::string id;
  // Longest matching pattern wins; equal-length ties preserve config order.
  RosResourcePattern pattern;
  // `std::nullopt` means auto for that axis.
  std::optional<rclcpp::ReliabilityPolicy> reliability;
  std::optional<rclcpp::DurabilityPolicy> durability;
};

struct SubscriptionQosConfig
{
  std::vector<TopicSubscriptionQosOverride> topic_overrides;
};

enum class SubscriptionQosResolutionSource
{
  Fallback,
  PublisherQos,
  Override
};

struct ResolvedSubscriptionQos
{
  // Resolution mutates only reliability and durability from the caller's base QoS.
  rclcpp::QoS qos{rclcpp::KeepLast(2)};
  // Stays `Override` when publisher QoS fills auto axes under a matching override.
  SubscriptionQosResolutionSource source = SubscriptionQosResolutionSource::Fallback;
  bool mixed_reliability = false;
  bool mixed_durability = false;
  std::size_t publisher_count = 0;
  std::string override_id;
};

// Per-axis precedence: override, discovered publisher QoS, then the base QoS.
// `publisher_qos` must come from one graph snapshot; `config` may be null.
ResolvedSubscriptionQos resolveSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base,
  const SubscriptionQosConfig * config,
  const std::vector<rclcpp::QoS> & publisher_qos);

// Throws if `graph` is null.
ResolvedSubscriptionQos resolveSubscriptionQos(
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & graph,
  std::string_view topic,
  const rclcpp::QoS & base,
  const SubscriptionQosConfig * config);

}  // namespace livekit_ros2_bridge
