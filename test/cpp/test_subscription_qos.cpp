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

#include "gtest/gtest.h"
#include "subscription_qos.hpp"

namespace livekit_ros2_bridge
{
namespace
{

rclcpp::QoS makeBaseQos(std::size_t depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

TEST(SubscriptionQosTest, FallsBackToBaseQosWithoutPublisherInfo)
{
  const rclcpp::QoS base_qos = makeBaseQos(10);
  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos("/camera/front", base_qos, nullptr, {});

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kFallback);
  EXPECT_FALSE(resolved.used_publisher_info);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, MixedPublisherReliabilityChoosesBestEffort)
{
  const rclcpp::QoS base_qos = makeBaseQos(10);
  const std::vector<ObservedPublisherQosProfile> publisher_profiles{
    {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::Volatile},
    {rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile},
  };

  const ResolvedSubscriptionQos resolved =
    resolveTopicSubscriptionQos("/camera/front", base_qos, nullptr, publisher_profiles);

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kInspection);
  EXPECT_TRUE(resolved.used_publisher_info);
  EXPECT_TRUE(resolved.mixed_reliability);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}

TEST(SubscriptionQosTest, MixedPublisherDurabilityChoosesVolatile)
{
  const rclcpp::QoS base_qos = makeBaseQos(10);
  const std::vector<ObservedPublisherQosProfile> publisher_profiles{
    {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal},
    {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::Volatile},
  };

  const ResolvedSubscriptionQos resolved =
    resolveTopicSubscriptionQos("/camera/front", base_qos, nullptr, publisher_profiles);

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kInspection);
  EXPECT_TRUE(resolved.used_publisher_info);
  EXPECT_TRUE(resolved.mixed_durability);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, LongestMatchingOverrideWinsAndAutoFieldsStillInspectPublishers)
{
  const rclcpp::QoS base_qos = makeBaseQos(10);

  SubscriptionQosConfig config;
  config.topic_overrides = {
    {"root", "/*", SubscriptionQosReliabilityMode::kBestEffort, SubscriptionQosDurabilityMode::kAuto},
    {"camera", "/camera/*", SubscriptionQosReliabilityMode::kAuto, SubscriptionQosDurabilityMode::kTransientLocal},
  };

  const std::vector<ObservedPublisherQosProfile> publisher_profiles{
    {rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile},
  };

  const ResolvedSubscriptionQos resolved =
    resolveTopicSubscriptionQos("/camera/front", base_qos, &config, publisher_profiles);

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kOverride);
  EXPECT_EQ(resolved.matched_override_id, "camera");
  EXPECT_EQ(resolved.matched_override_pattern, "/camera/*");
  EXPECT_TRUE(resolved.used_publisher_info);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

}  // namespace
}  // namespace livekit_ros2_bridge
