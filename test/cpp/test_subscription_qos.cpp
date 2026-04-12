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

rclcpp::QoS makeBaseQos()
{
  rclcpp::QoS qos{rclcpp::KeepLast(10)};
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

TEST(SubscriptionQosTest, FallsBackToBaseQosWithoutPublisherQos)
{
  const rclcpp::QoS base_qos = makeBaseQos();
  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos("/camera/front", base_qos, nullptr, {});

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kFallback);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, UnknownPublisherPoliciesDoNotOverrideBaseQos)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    nullptr,
    {
      {rclcpp::ReliabilityPolicy::Unknown, rclcpp::DurabilityPolicy::Unknown},
      {rclcpp::ReliabilityPolicy::SystemDefault, rclcpp::DurabilityPolicy::SystemDefault},
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kFallback);
  EXPECT_FALSE(resolved.used_publisher_qos);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, MixedPublisherReliabilityChoosesBestEffort)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    nullptr,
    {
      {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::Volatile},
      {rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile},
    });

  EXPECT_TRUE(resolved.mixed_reliability);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}

TEST(SubscriptionQosTest, MixedPublisherDurabilityChoosesVolatile)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    nullptr,
    {
      {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal},
      {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::Volatile},
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kPublisherQos);
  EXPECT_TRUE(resolved.mixed_durability);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, SinglePublisherQosInfersBothPoliciesWithoutMixedFlags)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  const ResolvedSubscriptionQos resolved = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    nullptr,
    {
      {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal},
    });

  EXPECT_TRUE(resolved.used_publisher_qos);
  EXPECT_FALSE(resolved.mixed_reliability);
  EXPECT_FALSE(resolved.mixed_durability);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(SubscriptionQosTest, OverrideAutoDurabilityUsesPublisherDurabilityWhenAvailableOtherwiseBaseDurability)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  SubscriptionQosConfig config;
  config.topic_overrides = {
    {"camera", "/camera/front", SubscriptionQosReliabilityMode::kBestEffort, SubscriptionQosDurabilityMode::kAuto},
  };

  const ResolvedSubscriptionQos without_publisher_durability = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    &config,
    {
      {rclcpp::ReliabilityPolicy::Unknown, rclcpp::DurabilityPolicy::Unknown},
    });

  const ResolvedSubscriptionQos with_publisher_durability = resolveTopicSubscriptionQos(
    "/camera/front",
    base_qos,
    &config,
    {
      {rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal},
    });

  EXPECT_EQ(without_publisher_durability.source, SubscriptionQosResolutionSource::kOverride);
  EXPECT_FALSE(without_publisher_durability.used_publisher_qos);
  EXPECT_EQ(without_publisher_durability.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(without_publisher_durability.qos.durability(), rclcpp::DurabilityPolicy::Volatile);

  EXPECT_EQ(with_publisher_durability.source, SubscriptionQosResolutionSource::kOverride);
  EXPECT_TRUE(with_publisher_durability.used_publisher_qos);
  EXPECT_EQ(with_publisher_durability.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(with_publisher_durability.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(SubscriptionQosTest, LongestMatchingOverrideWinsAndAutoReliabilityStillUsesPublisherQos)
{
  const rclcpp::QoS base_qos = makeBaseQos();

  SubscriptionQosConfig config;
  config.topic_overrides = {
    {"root", "/*", SubscriptionQosReliabilityMode::kBestEffort, SubscriptionQosDurabilityMode::kAuto},
    {"camera", "/camera/*", SubscriptionQosReliabilityMode::kAuto, SubscriptionQosDurabilityMode::kTransientLocal},
  };

  const std::vector<PublisherQosProfile> publisher_qos_profiles{
    {rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile},
  };

  const ResolvedSubscriptionQos resolved =
    resolveTopicSubscriptionQos("/camera/front", base_qos, &config, publisher_qos_profiles);

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::kOverride);
  EXPECT_EQ(resolved.matched_override_id, "camera");
  EXPECT_EQ(resolved.matched_override_pattern, "/camera/*");
  EXPECT_TRUE(resolved.used_publisher_qos);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

}  // namespace
}  // namespace livekit_ros2_bridge
