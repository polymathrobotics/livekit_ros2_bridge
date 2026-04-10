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
#include <future>
#include <stdexcept>

#include "gtest/gtest.h"
#include "utils/reentrant_quiesce_guard.hpp"

namespace livekit_ros2_bridge
{

constexpr auto kQuiesceStillBlockedWindow = std::chrono::milliseconds(50);
constexpr auto kQuiesceReadyTimeout = std::chrono::seconds(2);

TEST(ReentrantQuiesceGuardTest, TryBeginWorkSucceedsWhileEnabled)
{
  ReentrantQuiesceGuard guard;

  EXPECT_TRUE(guard.tryBeginWork());
  guard.endWork();
}

TEST(ReentrantQuiesceGuardTest, DisableBlocksNewTryBeginWorkCalls)
{
  ReentrantQuiesceGuard guard;

  guard.disable();

  EXPECT_FALSE(guard.tryBeginWork());
}

TEST(ReentrantQuiesceGuardTest, QuiesceBlocksUntilOtherThreadEndsWork)
{
  ReentrantQuiesceGuard guard;
  guard.beginWork();

  auto quiesce_future = std::async(std::launch::async, [&guard]() {
    guard.quiesce();
    return true;
  });

  EXPECT_EQ(quiesce_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  guard.endWork();

  EXPECT_EQ(quiesce_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  EXPECT_TRUE(quiesce_future.get());
}

TEST(ReentrantQuiesceGuardTest, QuiesceReturnsImmediatelyForOwningThread)
{
  ReentrantQuiesceGuard guard;
  guard.beginWork();

  const auto start = std::chrono::steady_clock::now();
  guard.quiesce();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, kQuiesceStillBlockedWindow);

  guard.endWork();
}

TEST(ReentrantQuiesceGuardTest, BeginWorkThrowsWhenSectionIsNotEnterable)
{
  ReentrantQuiesceGuard guard;
  guard.beginWork();

  EXPECT_THROW(guard.beginWork(), std::logic_error);

  guard.endWork();
  guard.disable();

  EXPECT_THROW(guard.beginWork(), std::logic_error);
}

}  // namespace livekit_ros2_bridge
