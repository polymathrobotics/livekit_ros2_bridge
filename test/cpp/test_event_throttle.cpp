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

#include <chrono>

#include "gtest/gtest.h"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

constexpr auto kThrottleInterval = std::chrono::milliseconds(20);

TEST(EventThrottleTest, FirstEventFiresImmediately)
{
  EventThrottle throttle(kThrottleInterval);

  EXPECT_EQ(throttle.record(), 1U);
}

TEST(EventThrottleTest, SuppressesEventsBeforeIntervalBoundary)
{
  EventThrottle throttle(std::chrono::hours(1));

  ASSERT_EQ(throttle.record(), 1U);
  EXPECT_EQ(throttle.record(), 0U);
}

TEST(EventThrottleTest, ZeroIntervalFiresOnEveryCall)
{
  EventThrottle throttle(std::chrono::milliseconds(0));

  EXPECT_EQ(throttle.record(), 1U);
  EXPECT_EQ(throttle.record(), 1U);
}

}  // namespace livekit_ros2_bridge
