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

namespace
{

class ManualSteadyClock
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  TimePoint now() const
  {
    return now_;
  }

  void advance(std::chrono::steady_clock::duration delta)
  {
    now_ += delta;
  }

private:
  TimePoint now_{};
};

}  // namespace

TEST(EventThrottleTest, FlushesSuppressedEventsOnlyAfterIntervalBoundary)
{
  ManualSteadyClock clock;
  EventThrottle throttle(kThrottleInterval, [&clock]() { return clock.now(); });

  ASSERT_EQ(throttle.recordAndTakePendingCount(), 1U);
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 0U);
  clock.advance(kThrottleInterval - std::chrono::milliseconds(1));
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 0U);

  clock.advance(std::chrono::milliseconds(1));
  ASSERT_EQ(throttle.recordAndTakePendingCount(), 3U);
}

TEST(EventThrottleTest, LateFlushRebasesThrottleWindowFromActualFireTime)
{
  ManualSteadyClock clock;
  EventThrottle throttle(kThrottleInterval, [&clock]() { return clock.now(); });
  const auto late_flush_skew = std::chrono::milliseconds(7);

  ASSERT_EQ(throttle.recordAndTakePendingCount(), 1U);

  clock.advance(kThrottleInterval * 5 + late_flush_skew);
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 1U);

  clock.advance(kThrottleInterval - std::chrono::milliseconds(1));
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 0U);

  clock.advance(std::chrono::milliseconds(1));
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 2U);
}

TEST(EventThrottleTest, ZeroIntervalFiresOnEveryCall)
{
  ManualSteadyClock clock;
  EventThrottle throttle(std::chrono::milliseconds(0), [&clock]() { return clock.now(); });

  EXPECT_EQ(throttle.recordAndTakePendingCount(), 1U);
  EXPECT_EQ(throttle.recordAndTakePendingCount(), 1U);
}

}  // namespace livekit_ros2_bridge
