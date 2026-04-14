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

TEST(ReentrantQuiesceGateTest, EnterThrowsWhileGateIsBusy)
{
  ReentrantQuiesceGate gate;

  ASSERT_TRUE(gate.tryEnter());
  EXPECT_THROW(gate.enter(), std::logic_error);

  gate.leave();
}

TEST(ReentrantQuiesceGateTest, AwaitIdleIsReentrantForOwnerButBlocksOtherThreads)
{
  ReentrantQuiesceGate gate;
  std::promise<void> owner_entered_promise;
  auto owner_entered_future = owner_entered_promise.get_future();
  std::promise<void> owner_await_idle_returned_promise;
  auto owner_await_idle_returned_future = owner_await_idle_returned_promise.get_future();
  std::promise<void> release_owner_promise;
  auto release_owner_future = release_owner_promise.get_future();

  auto owner_future = std::async(std::launch::async, [&]() {
    gate.enter();
    owner_entered_promise.set_value();

    gate.awaitIdle();
    owner_await_idle_returned_promise.set_value();

    release_owner_future.wait();
    gate.leave();
  });

  auto release_owner_and_join = [&]() {
    release_owner_promise.set_value();
    ASSERT_EQ(owner_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
    owner_future.get();
  };

  if (owner_entered_future.wait_for(kQuiesceReadyTimeout) != std::future_status::ready) {
    gate.leave();
    release_owner_and_join();
    FAIL() << "Owner thread did not enter the gate in time.";
  }

  auto other_thread_idle_future = std::async(std::launch::async, [&]() { gate.awaitIdle(); });

  EXPECT_EQ(other_thread_idle_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  if (owner_await_idle_returned_future.wait_for(kQuiesceReadyTimeout) != std::future_status::ready) {
    gate.leave();
    release_owner_and_join();
    ASSERT_EQ(other_thread_idle_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
    other_thread_idle_future.get();
    FAIL() << "Owning thread should not block when it reenters awaitIdle().";
  }

  release_owner_promise.set_value();

  ASSERT_EQ(other_thread_idle_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  other_thread_idle_future.get();

  ASSERT_EQ(owner_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  owner_future.get();
}

TEST(ReentrantQuiesceGateTest, CloseDuringActiveEntryWaitsForLeaveAndKeepsGateClosed)
{
  ReentrantQuiesceGate gate;
  std::promise<void> other_thread_started_promise;
  auto other_thread_started_future = other_thread_started_promise.get_future();

  gate.enter();
  gate.close();

  auto other_thread_idle_future = std::async(std::launch::async, [&]() {
    other_thread_started_promise.set_value();
    gate.awaitIdle();
  });

  ASSERT_EQ(other_thread_started_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  EXPECT_EQ(other_thread_idle_future.wait_for(kQuiesceStillBlockedWindow), std::future_status::timeout);

  gate.leave();

  ASSERT_EQ(other_thread_idle_future.wait_for(kQuiesceReadyTimeout), std::future_status::ready);
  other_thread_idle_future.get();

  EXPECT_FALSE(gate.tryEnter());
  EXPECT_THROW(gate.enter(), std::logic_error);
}

}  // namespace livekit_ros2_bridge
