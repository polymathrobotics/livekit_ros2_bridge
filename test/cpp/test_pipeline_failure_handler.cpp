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
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

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

TEST(PipelineFailureHandlerTest, ScheduleFromInsideCallbackIsIgnored)
{
  // A failure delivered while the callback runs (the restart's own state change)
  // must not queue another run; callers that want a retry ask for it explicitly.
  std::mutex mutex;
  std::condition_variable condition;
  std::unique_ptr<PipelineFailureHandler> handler;
  int callback_count = 0;
  bool reschedule_checked = false;
  bool reschedule_accepted = true;
  handler = std::make_unique<PipelineFailureHandler>(std::chrono::milliseconds::zero(), [&]() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++callback_count;
    }
    const bool accepted = handler->schedule();
    {
      std::lock_guard<std::mutex> lock(mutex);
      reschedule_accepted = accepted;
      reschedule_checked = true;
      condition.notify_all();
    }
  });

  EXPECT_TRUE(handler->schedule());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return reschedule_checked; }));
  }

  // Give an unexpected rerun a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  handler->close();

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(callback_count, 1);
  EXPECT_FALSE(reschedule_accepted);
}

TEST(PipelineFailureHandlerTest, ScheduleFromAnotherThreadWhileCallbackRunsIsIgnored)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(20), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));

  EXPECT_FALSE(handler.schedule());
  callback.releaseFirstRun();

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(callback.runCount(), 1);

  // Once the callback has returned, a new failure schedules a run again.
  EXPECT_TRUE(handler.schedule());
  EXPECT_TRUE(callback.waitForRunCount(2));

  handler.close();
}

// Records each run's start time; on the first run the callback calls request
// (e.g. rerunAfterCurrentRun()) through the handler under test.
class RecordingCallback
{
public:
  void operator()(const std::function<void()> & request_on_first_run)
  {
    bool first_run = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      run_times_.push_back(std::chrono::steady_clock::now());
      first_run = run_times_.size() == 1U;
    }
    if (first_run) {
      request_on_first_run();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    condition_.notify_all();
  }

  bool waitForRunCount(std::size_t expected_count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [&]() { return run_times_.size() >= expected_count; });
  }

  std::vector<std::chrono::steady_clock::time_point> runTimes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return run_times_;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<std::chrono::steady_clock::time_point> run_times_;
};

TEST(PipelineFailureHandlerTest, RerunAfterCurrentRunRunsCallbackOnceMoreAfterTheDelay)
{
  constexpr auto kDelay = std::chrono::milliseconds(100);
  RecordingCallback callback;
  std::unique_ptr<PipelineFailureHandler> handler;
  handler =
    std::make_unique<PipelineFailureHandler>(kDelay, [&]() { callback([&]() { handler->rerunAfterCurrentRun(); }); });

  EXPECT_TRUE(handler->schedule());
  ASSERT_TRUE(callback.waitForRunCount(2U));

  // Give an unexpected third run a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  handler->close();

  const auto run_times = callback.runTimes();
  ASSERT_EQ(run_times.size(), 2U);
  EXPECT_GE(run_times[1] - run_times[0], kDelay);
}

TEST(PipelineFailureHandlerTest, RepeatedRerunRequestsCoalesceIntoOneRun)
{
  RecordingCallback callback;
  bool reschedule_accepted = true;
  std::unique_ptr<PipelineFailureHandler> handler;
  handler = std::make_unique<PipelineFailureHandler>(std::chrono::milliseconds(20), [&]() {
    callback([&]() {
      handler->rerunAfterCurrentRun();
      handler->rerunAfterCurrentRun();
      reschedule_accepted = handler->schedule();
      handler->rerunAfterCurrentRun();
    });
  });

  EXPECT_TRUE(handler->schedule());
  ASSERT_TRUE(callback.waitForRunCount(2U));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  handler->close();

  EXPECT_EQ(callback.runTimes().size(), 2U);
  EXPECT_FALSE(reschedule_accepted);
}

TEST(PipelineFailureHandlerTest, ScheduleFoldsIntoARequestedRerun)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(300), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));
  handler.rerunAfterCurrentRun();
  callback.releaseFirstRun();

  // Lets the callback return; the rerun now waits out the delay.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(handler.schedule());

  ASSERT_TRUE(callback.waitForRunCount(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_EQ(callback.runCount(), 2);

  handler.close();
}

TEST(PipelineFailureHandlerTest, CancelPendingWhileCallbackRunsDropsRequestedRerun)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(20), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));
  handler.rerunAfterCurrentRun();
  handler.cancelPending();
  callback.releaseFirstRun();

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(callback.runCount(), 1);

  handler.close();
}

TEST(PipelineFailureHandlerTest, CancelPendingDuringTheDelayDropsRequestedRerun)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(300), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));
  handler.rerunAfterCurrentRun();
  callback.releaseFirstRun();

  // Lets the callback return; the rerun now waits out the delay.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  handler.cancelPending();

  std::this_thread::sleep_for(std::chrono::milliseconds(450));
  EXPECT_EQ(callback.runCount(), 1);

  handler.close();
}

TEST(PipelineFailureHandlerTest, CloseWhileCallbackRunsDropsRequestedRerun)
{
  BlockingCallback callback;
  // The rerun waits out the delay after the running callback returns, so close()
  // has that long to land even if the closer thread starts late; either way the
  // requested rerun must be dropped and close() must return.
  PipelineFailureHandler handler(std::chrono::milliseconds(500), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));
  handler.rerunAfterCurrentRun();

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
}

TEST(PipelineFailureHandlerTest, RerunAfterCurrentRunIsIgnoredOnceClosed)
{
  BlockingCallback callback;
  PipelineFailureHandler handler(std::chrono::milliseconds(20), [&]() { callback(); });

  EXPECT_TRUE(handler.schedule());
  ASSERT_TRUE(callback.waitForRunCount(1));

  std::thread closer([&]() { handler.close(); });
  // Usually lets close() mark the handler closed while the callback still runs.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  handler.rerunAfterCurrentRun();
  callback.releaseFirstRun();
  closer.join();

  handler.rerunAfterCurrentRun();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(callback.runCount(), 1);
}

TEST(PipelineFailureHandlerTest, RerunAfterCurrentRunOutsideTheCallbackDoesNothing)
{
  std::atomic<int> callback_count{0};
  PipelineFailureHandler handler(std::chrono::milliseconds(200), [&]() { callback_count.fetch_add(1); });

  // Before any run (no worker thread yet): no run starts, and nothing is left
  // pending that would swallow the next schedule().
  handler.rerunAfterCurrentRun();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(callback_count.load(), 0);

  // While a run waits out its delay: the request does not add a second run.
  EXPECT_TRUE(handler.schedule());
  handler.rerunAfterCurrentRun();

  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  EXPECT_EQ(callback_count.load(), 1);

  handler.close();
}

}  // namespace
}  // namespace livekit_ros2_bridge::utils
