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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "utils/pipeline_failure_handler.hpp"

namespace livekit_ros2_bridge::utils
{
namespace
{

// These tests exercise the coalescing and shutdown contract directly; they use
// short delays so the bounded waits stay well under the suite timeout. The
// handler joins its worker on close/destruction, so a passing test also proves
// teardown does not hang.

TEST(PipelineFailureHandlerTest, ScheduleRunsCallbackOnceAndCoalesces)
{
  std::mutex mutex;
  std::condition_variable condition;
  int callback_count = 0;

  PipelineFailureHandler handler(std::chrono::milliseconds(30), [&]() {
    std::lock_guard<std::mutex> lock(mutex);
    ++callback_count;
    condition.notify_all();
  });

  EXPECT_TRUE(handler.schedule());
  // The second schedule lands while the first is still pending; it must be
  // folded into that one callback, not queue another.
  EXPECT_FALSE(handler.schedule());

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return callback_count == 1; }));
  }

  // Give a coalesced duplicate a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(callback_count, 1);
  }

  handler.close();
}

TEST(PipelineFailureHandlerTest, ClosedHandlerRejectsScheduleAndJoinsOnDestruction)
{
  EXPECT_NO_THROW({
    PipelineFailureHandler handler(std::chrono::milliseconds(10), []() {});
    handler.close();
    EXPECT_FALSE(handler.schedule());
  });
}

TEST(PipelineFailureHandlerTest, CancelPendingPreventsCallback)
{
  std::atomic<int> callback_count{0};
  PipelineFailureHandler handler(std::chrono::milliseconds(200), [&]() { callback_count.fetch_add(1); });

  EXPECT_TRUE(handler.schedule());
  handler.cancelPending();

  // Wait past the scheduled delay: a cancelled schedule never fires.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_EQ(callback_count.load(), 0);

  handler.close();
}

}  // namespace
}  // namespace livekit_ros2_bridge::utils
