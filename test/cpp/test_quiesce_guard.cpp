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

#include "gtest/gtest.h"
#include "utils/quiesce_guard.hpp"

namespace livekit_ros2_bridge
{

TEST(QuiesceGuardTest, InitialGenerationIsZero)
{
  QuiesceGuard guard;

  EXPECT_EQ(guard.currentGeneration(), 0U);
}

TEST(QuiesceGuardTest, TryBeginWorkSucceedsForCurrentGenerationWhileEnabled)
{
  QuiesceGuard guard;
  const std::size_t generation = guard.currentGeneration();

  EXPECT_TRUE(guard.tryBeginWork(generation));

  guard.endWork();
}

TEST(QuiesceGuardTest, QuiesceBlocksUntilActiveWorkEnds)
{
  QuiesceGuard guard;
  const std::size_t initial_generation = guard.currentGeneration();
  ASSERT_TRUE(guard.tryBeginWork(initial_generation));

  auto quiesce_future = std::async(std::launch::async, [&guard]() { return guard.quiesce(); });

  EXPECT_EQ(quiesce_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  EXPECT_FALSE(guard.tryBeginWork(initial_generation));

  guard.endWork();

  EXPECT_EQ(quiesce_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(quiesce_future.get(), initial_generation + 1U);
}

TEST(QuiesceGuardTest, OldGenerationIsRejectedAfterQuiesceAndResume)
{
  QuiesceGuard guard;
  const std::size_t old_generation = guard.currentGeneration();

  const std::size_t new_generation = guard.quiesce();

  EXPECT_EQ(new_generation, old_generation + 1U);
  EXPECT_FALSE(guard.tryBeginWork(old_generation));
  EXPECT_FALSE(guard.tryBeginWork(new_generation));

  guard.resume(new_generation);

  EXPECT_TRUE(guard.tryBeginWork(new_generation));
  guard.endWork();
}

TEST(QuiesceGuardTest, StaleResumeIsNoOp)
{
  QuiesceGuard guard;
  const std::size_t old_generation = guard.currentGeneration();
  const std::size_t new_generation = guard.quiesce();

  guard.resume(old_generation);
  EXPECT_FALSE(guard.tryBeginWork(new_generation));

  guard.resume(new_generation);
  EXPECT_TRUE(guard.tryBeginWork(new_generation));
  guard.endWork();
}

}  // namespace livekit_ros2_bridge
