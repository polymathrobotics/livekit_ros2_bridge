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

#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "subscription_qos.hpp"

namespace livekit_ros2_bridge
{
namespace
{

rclcpp::QoS makePublisherQos(rclcpp::ReliabilityPolicy reliability, rclcpp::DurabilityPolicy durability)
{
  rclcpp::QoS qos{10};
  qos.reliability(reliability);
  qos.durability(durability);
  return qos;
}

TopicSubscriptionQosOverride makeOverride(
  const char * id,
  const char * pattern,
  std::optional<rclcpp::ReliabilityPolicy> reliability,
  std::optional<rclcpp::DurabilityPolicy> durability)
{
  TopicSubscriptionQosOverride qos_override;
  qos_override.id = id;
  qos_override.pattern = RosResourcePattern::fromCanonical(pattern);
  qos_override.reliability = reliability;
  qos_override.durability = durability;
  return qos_override;
}

TEST(SubscriptionQosTest, FallsBackToBaseQosWithoutPublisherQos)
{
  const rclcpp::QoS base{10};
  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos("/camera/front", base, nullptr, {});

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::Fallback);
  EXPECT_EQ(resolved.publisher_count, 0U);
  EXPECT_EQ(resolved.qos.reliability(), base.reliability());
  EXPECT_EQ(resolved.qos.durability(), base.durability());
}

TEST(SubscriptionQosTest, UnknownPublisherPoliciesDoNotOverrideBaseQos)
{
  const rclcpp::QoS base{10};

  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos(
    "/camera/front",
    base,
    nullptr,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::Unknown, rclcpp::DurabilityPolicy::Unknown),
      makePublisherQos(rclcpp::ReliabilityPolicy::SystemDefault, rclcpp::DurabilityPolicy::SystemDefault),
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::Fallback);
  EXPECT_EQ(resolved.publisher_count, 2U);
  EXPECT_EQ(resolved.qos.reliability(), base.reliability());
  EXPECT_EQ(resolved.qos.durability(), base.durability());
}

TEST(SubscriptionQosTest, MixedPublisherPoliciesChooseWeakerCompatiblePolicyPerAxis)
{
  const rclcpp::QoS base{10};

  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos(
    "/camera/front",
    base,
    nullptr,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal),
      makePublisherQos(rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile),
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::PublisherQos);
  EXPECT_TRUE(resolved.mixed_reliability);
  EXPECT_TRUE(resolved.mixed_durability);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(SubscriptionQosTest, SinglePublisherQosInfersBothPoliciesWithoutMixedFlags)
{
  const rclcpp::QoS base{10};

  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos(
    "/camera/front",
    base,
    nullptr,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal),
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::PublisherQos);
  EXPECT_FALSE(resolved.mixed_reliability);
  EXPECT_FALSE(resolved.mixed_durability);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(SubscriptionQosTest, PublisherQosOnlyOverridesKnownAxisAndKeepsBaseForOtherAxis)
{
  const rclcpp::QoS base{10};

  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos(
    "/camera/front",
    base,
    nullptr,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Unknown),
      makePublisherQos(rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::SystemDefault),
    });

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::PublisherQos);
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(resolved.qos.durability(), base.durability());
}

TEST(SubscriptionQosTest, OverrideAutoDurabilityUsesPublisherDurabilityWhenAvailableOtherwiseBaseDurability)
{
  const rclcpp::QoS base{10};

  SubscriptionQosConfig config;
  config.topic_overrides = {
    makeOverride("camera", "/camera/front", rclcpp::ReliabilityPolicy::BestEffort, std::nullopt),
  };

  const ResolvedSubscriptionQos without_publisher_durability = resolveSubscriptionQos(
    "/camera/front",
    base,
    &config,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::Unknown, rclcpp::DurabilityPolicy::Unknown),
    });

  const ResolvedSubscriptionQos with_publisher_durability = resolveSubscriptionQos(
    "/camera/front",
    base,
    &config,
    {
      makePublisherQos(rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::TransientLocal),
    });

  EXPECT_EQ(without_publisher_durability.source, SubscriptionQosResolutionSource::Override);
  EXPECT_EQ(without_publisher_durability.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(without_publisher_durability.qos.durability(), base.durability());

  EXPECT_EQ(with_publisher_durability.source, SubscriptionQosResolutionSource::Override);
  EXPECT_EQ(with_publisher_durability.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(with_publisher_durability.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(SubscriptionQosTest, LongestMatchingOverrideWinsAndAutoReliabilityStillUsesPublisherQos)
{
  const rclcpp::QoS base{10};

  SubscriptionQosConfig config;
  config.topic_overrides = {
    makeOverride("root", "/*", rclcpp::ReliabilityPolicy::BestEffort, std::nullopt),
    makeOverride("camera", "/camera/*", std::nullopt, rclcpp::DurabilityPolicy::TransientLocal),
  };

  const std::vector<rclcpp::QoS> publishers{
    makePublisherQos(rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile),
  };

  const ResolvedSubscriptionQos resolved = resolveSubscriptionQos("/camera/front", base, &config, publishers);

  EXPECT_EQ(resolved.source, SubscriptionQosResolutionSource::Override);
  EXPECT_EQ(resolved.override_id, "camera");
  EXPECT_EQ(resolved.qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(resolved.qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

}  // namespace
}  // namespace livekit_ros2_bridge
