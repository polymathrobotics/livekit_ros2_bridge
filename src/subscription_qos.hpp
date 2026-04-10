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

#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

namespace livekit_ros2_bridge
{

enum class SubscriptionQosReliabilityMode
{
  kAuto,
  kReliable,
  kBestEffort
};

enum class SubscriptionQosDurabilityMode
{
  kAuto,
  kVolatile,
  kTransientLocal
};

struct TopicSubscriptionQosOverride
{
  std::string id;
  std::string pattern;
  SubscriptionQosReliabilityMode reliability = SubscriptionQosReliabilityMode::kAuto;
  SubscriptionQosDurabilityMode durability = SubscriptionQosDurabilityMode::kAuto;
};

struct SubscriptionQosConfig
{
  std::vector<TopicSubscriptionQosOverride> topic_overrides;
};

struct ObservedPublisherQosProfile
{
  rclcpp::ReliabilityPolicy reliability = rclcpp::ReliabilityPolicy::Unknown;
  rclcpp::DurabilityPolicy durability = rclcpp::DurabilityPolicy::Unknown;
};

enum class SubscriptionQosResolutionSource
{
  kFallback,
  kInspection,
  kOverride
};

struct ResolvedSubscriptionQos
{
  rclcpp::QoS qos{rclcpp::KeepLast(2)};
  SubscriptionQosResolutionSource source = SubscriptionQosResolutionSource::kFallback;
  bool used_publisher_info = false;
  bool mixed_reliability = false;
  bool mixed_durability = false;
  std::string matched_override_id;
  std::string matched_override_pattern;
};

ResolvedSubscriptionQos resolveTopicSubscriptionQos(
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config,
  const std::vector<ObservedPublisherQosProfile> & publisher_profiles);

ResolvedSubscriptionQos resolveTopicSubscriptionQos(
  const rclcpp::Node & node,
  std::string_view topic,
  const rclcpp::QoS & base_qos,
  const SubscriptionQosConfig * config);

const char * subscriptionQosResolutionSourceToString(SubscriptionQosResolutionSource source);
const char * reliabilityPolicyToString(rclcpp::ReliabilityPolicy policy);
const char * durabilityPolicyToString(rclcpp::DurabilityPolicy policy);

}  // namespace livekit_ros2_bridge
