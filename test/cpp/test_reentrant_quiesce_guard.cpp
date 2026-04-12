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
#include "utils/reentrant_quiesce_gate.hpp"

namespace livekit_ros2_bridge
{

constexpr auto kQuiesceStillBlockedWindow = std::chrono::milliseconds(50);
constexpr auto kQuiesceReadyTimeout = std::chrono::seconds(2);

TEST(ReentrantQuiesceGateTest, TryEnterSucceedsWhileOpen)
{
  ReentrantQuiesceGate gate;

  EXPECT_TRUE(gate.tryEnter());
  gate.leave();
}

TEST(ReentrantQuiesceGateTest, CloseBlocksNewEntries)
{
  ReentrantQuiesceGate gate;

  gate.close();

  EXPECT_FALSE(gate.tryEnter());
}

TEST(ReentrantQuiesceGateTest, AwaitIdleBlocksUntilOtherThreadLeaves)
{
  ReentrantQuiesceGate gate;
  gate.enter();

  auto idle_future = std::async(std::launch::async, [&gate]() {
    gate.awaitIdle();
    return true;
  });

  EXPECT_EQ(idle_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  gate.leave();

  EXPECT_EQ(idle_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  EXPECT_TRUE(idle_future.get());
}

TEST(ReentrantQuiesceGateTest, AwaitIdleReturnsImmediatelyForOwningThread)
{
  ReentrantQuiesceGate gate;
  gate.enter();

  const auto start = std::chrono::steady_clock::now();
  gate.awaitIdle();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, kQuiesceStillBlockedWindow);

  gate.leave();
}

TEST(ReentrantQuiesceGateTest, EnterThrowsWhenSectionIsNotEnterable)
{
  ReentrantQuiesceGate gate;
  gate.enter();

  EXPECT_THROW(gate.enter(), std::logic_error);

  gate.leave();
  gate.close();

  EXPECT_THROW(gate.enter(), std::logic_error);
}

}  // namespace livekit_ros2_bridge
