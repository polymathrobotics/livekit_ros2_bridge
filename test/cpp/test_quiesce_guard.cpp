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
#include <thread>

#include "gtest/gtest.h"
#include "utils/quiesce_gate.hpp"

namespace livekit_ros2_bridge
{

constexpr auto kQuiesceStillBlockedWindow = std::chrono::milliseconds(50);
constexpr auto kQuiesceReadyTimeout = std::chrono::seconds(2);

namespace
{

bool waitForGeneration(QuiesceGate & gate, std::size_t expected_generation)
{
  const auto deadline = std::chrono::steady_clock::now() + kQuiesceReadyTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (gate.currentGeneration() == expected_generation) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return gate.currentGeneration() == expected_generation;
}

}  // namespace

TEST(QuiesceGateTest, CloseBlocksNewEntriesAndWaitsForAllActiveEntriesToLeave)
{
  QuiesceGate gate;
  const std::size_t initial_generation = gate.currentGeneration();
  ASSERT_TRUE(gate.tryEnter(initial_generation));
  ASSERT_TRUE(gate.tryEnter(initial_generation));

  auto close_future = std::async(std::launch::async, [&gate]() { return gate.close(); });

  EXPECT_EQ(close_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);
  EXPECT_FALSE(gate.tryEnter(initial_generation));

  gate.leave();

  EXPECT_EQ(close_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  gate.leave();

  ASSERT_EQ(close_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  EXPECT_EQ(close_future.get(), initial_generation + 1U);
}

TEST(QuiesceGateTest, OpenRequiresCurrentGenerationAcrossRepeatedClose)
{
  QuiesceGate gate;
  const std::size_t initial_generation = gate.currentGeneration();
  const std::size_t next_generation = gate.close();

  EXPECT_FALSE(gate.tryEnter(next_generation));

  gate.open(next_generation);

  EXPECT_FALSE(gate.tryEnter(initial_generation));
  ASSERT_TRUE(gate.tryEnter(next_generation));
  gate.leave();

  const std::size_t latest_generation = gate.close();

  EXPECT_EQ(latest_generation, next_generation + 1U);

  gate.open(next_generation);
  EXPECT_FALSE(gate.tryEnter(latest_generation));

  gate.open(latest_generation);
  EXPECT_TRUE(gate.tryEnter(latest_generation));
  gate.leave();
}

TEST(QuiesceGateTest, OpenDoesNotReAdmitWorkBeforeCloseFinishesDraining)
{
  QuiesceGate gate;
  const std::size_t initial_generation = gate.currentGeneration();
  ASSERT_TRUE(gate.tryEnter(initial_generation));

  auto close_future = std::async(std::launch::async, [&gate]() { return gate.close(); });

  const std::size_t draining_generation = initial_generation + 1U;
  ASSERT_TRUE(waitForGeneration(gate, draining_generation));
  EXPECT_EQ(close_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  gate.open(draining_generation);

  const bool entered_during_drain = gate.tryEnter(draining_generation);
  EXPECT_FALSE(entered_during_drain);
  if (entered_during_drain) {
    gate.leave();
  }

  gate.leave();

  ASSERT_EQ(close_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  EXPECT_EQ(close_future.get(), draining_generation);

  gate.open(draining_generation);
  ASSERT_TRUE(gate.tryEnter(draining_generation));
  gate.leave();
}

}  // namespace livekit_ros2_bridge
