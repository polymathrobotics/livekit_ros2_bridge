// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
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

// Lets a test hold the handler's callback mid-run so it can call schedule()
// or close() while the callback is still running. Every wait is bounded so a
// regression fails the test instead of hanging the suite.
class BlockingCallback
{
public:
  void operator()()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++run_count_;
    condition_.notify_all();
    if (run_count_ == 1) {
      condition_.wait_for(lock, std::chrono::seconds(5), [this]() { return released_; });
    }
  }

  bool waitForRunCount(int expected_count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [&]() { return run_count_ >= expected_count; });
  }

  void releaseFirstRun()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  int runCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return run_count_;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  int run_count_ = 0;
  bool released_ = false;
};

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

TEST(PipelineFailureHandlerTest, ScheduleWhileCallbackRunsRunsCallbackOnceMore)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(20), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));

  // A failure posted while the restart runs (e.g. the restarted pipeline fails
  // at once) must not be dropped; the first such request is reported as newly
  // scheduled so the caller logs the follow-up recovery.
  EXPECT_TRUE(handler.schedule());
  callback.releaseFirstRun();

  ASSERT_TRUE(callback.waitForRunCount(2));
  // Give an unexpected third run a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(callback.runCount(), 2);

  handler.close();
}

TEST(PipelineFailureHandlerTest, RepeatedScheduleWhileCallbackRunsCoalescesIntoOneRerun)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(20), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));

  EXPECT_TRUE(handler.schedule());
  EXPECT_FALSE(handler.schedule());
  EXPECT_FALSE(handler.schedule());
  callback.releaseFirstRun();

  ASSERT_TRUE(callback.waitForRunCount(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(callback.runCount(), 2);

  handler.close();
}

TEST(PipelineFailureHandlerTest, ScheduleFromInsideCallbackRunsCallbackOnceMore)
{
  // GStreamer can deliver a failure synchronously while the restart callback
  // starts the new pipeline, so schedule() is called on the worker thread itself.
  std::mutex mutex;
  std::condition_variable condition;
  int callback_count = 0;
  bool first_reschedule_accepted = false;
  std::unique_ptr<PipelineFailureHandler> handler;
  handler = std::make_unique<PipelineFailureHandler>(std::chrono::milliseconds(20), [&]() {
    int run_number = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      run_number = ++callback_count;
    }
    if (run_number == 1) {
      const bool accepted = handler->schedule();
      std::lock_guard<std::mutex> lock(mutex);
      first_reschedule_accepted = accepted;
    }
    std::lock_guard<std::mutex> lock(mutex);
    condition.notify_all();
  });

  EXPECT_TRUE(handler->schedule());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return callback_count == 2; }));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  handler->close();

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(callback_count, 2);
  EXPECT_TRUE(first_reschedule_accepted);
}

TEST(PipelineFailureHandlerTest, CloseWhileCallbackRunsDropsPendingRerun)
{
  BlockingCallback callback;
  // The rerun waits out the delay after the running callback returns, so close()
  // has that long to land even if the closer thread starts late; either way the
  // pending rerun must be cancelled and close() must return.
  PipelineFailureHandler handler(std::chrono::milliseconds(500), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));
  EXPECT_TRUE(handler.schedule());

  std::mutex mutex;
  std::condition_variable condition;
  bool close_returned = false;
  std::thread closer([&]() {
    handler.close();
    std::lock_guard<std::mutex> lock(mutex);
    close_returned = true;
    condition.notify_all();
  });

  // Usually lets close() mark the handler closed while the callback still runs.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  callback.releaseFirstRun();

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return close_returned; }));
  }
  closer.join();

  EXPECT_EQ(callback.runCount(), 1);
  EXPECT_FALSE(handler.schedule());
}

}  // namespace
}  // namespace livekit_ros2_bridge::utils
