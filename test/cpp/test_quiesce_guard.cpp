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

TEST(QuiesceGateTest, InitialGenerationIsZero)
{
  QuiesceGate gate;

  EXPECT_EQ(gate.currentGeneration(), 0U);
}

TEST(QuiesceGateTest, TryEnterSucceedsForCurrentGenerationWhileOpen)
{
  QuiesceGate gate;
  const std::size_t generation = gate.currentGeneration();

  EXPECT_TRUE(gate.tryEnter(generation));

  gate.leave();
}

TEST(QuiesceGateTest, CloseBlocksUntilActiveEntriesLeave)
{
  QuiesceGate gate;
  const std::size_t initial_generation = gate.currentGeneration();
  ASSERT_TRUE(gate.tryEnter(initial_generation));

  auto close_future = std::async(std::launch::async, [&gate]() { return gate.close(); });

  EXPECT_EQ(close_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  EXPECT_FALSE(gate.tryEnter(initial_generation));

  gate.leave();

  EXPECT_EQ(close_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(close_future.get(), initial_generation + 1U);
}

TEST(QuiesceGateTest, OldGenerationIsRejectedAfterCloseAndOpen)
{
  QuiesceGate gate;
  const std::size_t old_generation = gate.currentGeneration();

  const std::size_t new_generation = gate.close();

  EXPECT_EQ(new_generation, old_generation + 1U);
  EXPECT_FALSE(gate.tryEnter(old_generation));
  EXPECT_FALSE(gate.tryEnter(new_generation));

  gate.open(new_generation);

  EXPECT_TRUE(gate.tryEnter(new_generation));
  gate.leave();
}

TEST(QuiesceGateTest, StaleOpenIsNoOp)
{
  QuiesceGate gate;
  const std::size_t old_generation = gate.currentGeneration();
  const std::size_t new_generation = gate.close();

  gate.open(old_generation);
  EXPECT_FALSE(gate.tryEnter(new_generation));

  gate.open(new_generation);
  EXPECT_TRUE(gate.tryEnter(new_generation));
  gate.leave();
}

}  // namespace livekit_ros2_bridge
