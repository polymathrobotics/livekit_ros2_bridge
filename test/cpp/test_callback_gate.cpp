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
#include "utils/callback_gate.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(CallbackGateTest, CloseRejectsNewCallbacksAndReportsFirstClose)
{
  CallbackGate gate;

  bool ran = false;
  EXPECT_TRUE(gate.run([&ran]() { ran = true; }));
  EXPECT_TRUE(ran);

  EXPECT_TRUE(gate.closeAndWait());
  EXPECT_TRUE(gate.isClosed());

  ran = false;
  EXPECT_FALSE(gate.run([&ran]() { ran = true; }));
  EXPECT_FALSE(ran);
  EXPECT_FALSE(gate.closeAndWait());
}

TEST(CallbackGateTest, CloseWaitsForActiveCallbacks)
{
  CallbackGate gate;
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future().share();

  std::thread worker([&gate, &entered, release_future]() {
    EXPECT_TRUE(gate.run([&entered, release_future]() {
      entered.set_value();
      release_future.wait();
    }));
  });

  const auto entered_status = entered_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(entered_status, std::future_status::ready);
  if (entered_status != std::future_status::ready) {
    release.set_value();
    worker.join();
    return;
  }

  auto close_future = std::async(std::launch::async, [&gate]() { return gate.closeAndWait(); });
  EXPECT_EQ(close_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  release.set_value();
  EXPECT_EQ(close_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_TRUE(close_future.get());
  worker.join();
}

TEST(CallbackGateTest, ExclusiveModeRejectsOverlappingCallbacks)
{
  CallbackGate gate(CallbackGate::Concurrency::Exclusive);
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future().share();

  std::thread worker([&gate, &entered, release_future]() {
    EXPECT_TRUE(gate.run([&entered, release_future]() {
      entered.set_value();
      release_future.wait();
    }));
  });

  const auto entered_status = entered_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(entered_status, std::future_status::ready);
  if (entered_status != std::future_status::ready) {
    release.set_value();
    worker.join();
    return;
  }

  bool overlapped = false;
  EXPECT_FALSE(gate.run([&overlapped]() { overlapped = true; }));
  EXPECT_FALSE(overlapped);

  release.set_value();
  worker.join();

  bool ran_after_idle = false;
  EXPECT_TRUE(gate.run([&ran_after_idle]() { ran_after_idle = true; }));
  EXPECT_TRUE(ran_after_idle);
}

TEST(CallbackGateTest, ExcludingCurrentThreadDoesNotWaitForCurrentCallback)
{
  CallbackGate gate(CallbackGate::Concurrency::Exclusive);
  bool close_returned = false;

  EXPECT_TRUE(gate.run([&gate, &close_returned]() {
    close_returned = gate.closeAndWait(CallbackGate::WaitMode::ExcludingCurrentThread);
  }));

  EXPECT_TRUE(close_returned);
  EXPECT_FALSE(gate.run([]() {}));
}

TEST(CallbackGateTest, ExcludingCurrentThreadStillWaitsForOtherThreads)
{
  CallbackGate gate(CallbackGate::Concurrency::Exclusive);
  std::promise<void> worker_entered;
  auto worker_entered_future = worker_entered.get_future();
  std::promise<void> release_worker;
  auto release_worker_future = release_worker.get_future().share();

  std::thread worker([&gate, &worker_entered, release_worker_future]() {
    EXPECT_TRUE(gate.run([&worker_entered, release_worker_future]() {
      worker_entered.set_value();
      release_worker_future.wait();
    }));
  });

  const auto worker_entered_status = worker_entered_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(worker_entered_status, std::future_status::ready);
  if (worker_entered_status != std::future_status::ready) {
    release_worker.set_value();
    worker.join();
    return;
  }

  auto close_future = std::async(
    std::launch::async, [&gate]() { return gate.closeAndWait(CallbackGate::WaitMode::ExcludingCurrentThread); });
  EXPECT_EQ(close_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  release_worker.set_value();
  EXPECT_EQ(close_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_TRUE(close_future.get());
  worker.join();
}

}  // namespace

}  // namespace livekit_ros2_bridge
