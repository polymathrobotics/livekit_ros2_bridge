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
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "support/detached_task.hpp"
#include "utils/pipeline_failure_handler.hpp"

namespace livekit_ros2_bridge::utils
{
namespace
{

// These tests exercise the coalescing and shutdown contract directly; they use short delays
// so the bounded waits stay well under the suite timeout. Calls that could deadlock go
// through finishesWithin, so a hang fails the test instead of hanging the binary.

// How long a call that could deadlock gets before the test gives up on it.
constexpr auto kHangTimeout = std::chrono::seconds(2);

// Stands in for a stream's restart callback and records when each run starts. The first run
// calls the first-run action, then optionally waits for releaseFirstRun() so the test can act
// while the callback runs. Every wait is bounded so a regression fails instead of hanging.
class RecordingCallback
{
public:
  void operator()()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    run_times_.push_back(std::chrono::steady_clock::now());
    condition_.notify_all();
    if (run_times_.size() != 1U) {
      return;
    }

    const std::function<void()> first_run_action = first_run_action_;
    lock.unlock();
    if (first_run_action) {
      first_run_action();
    }
    lock.lock();
    if (hold_first_run_) {
      condition_.wait_for(lock, std::chrono::seconds(5), [this]() { return released_; });
    }
  }

  // Both setters must be called before the handler's first schedule() or requestRun().
  void setFirstRunAction(std::function<void()> action)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    first_run_action_ = std::move(action);
  }

  void holdFirstRun()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_first_run_ = true;
  }

  void releaseFirstRun()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  bool waitForRunCount(std::size_t expected_count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(2), [&]() { return run_times_.size() >= expected_count; });
  }

  std::size_t runCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return run_times_.size();
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
  std::function<void()> first_run_action_;
  bool hold_first_run_ = false;
  bool released_ = false;
};

// Heap-owned by the test and by every call it hands to finishesWithin, so a call that is
// abandoned keeps the handler and the callback alive instead of touching destroyed ones.
struct HandlerHarness
{
  explicit HandlerHarness(std::chrono::milliseconds delay)
  : handler(delay, [this]() { callback(); })
  {}

  RecordingCallback callback;
  PipelineFailureHandler handler;
};

class PipelineFailureHandlerTest : public ::testing::Test
{
protected:
  std::shared_ptr<HandlerHarness> makeHarness(std::chrono::milliseconds delay)
  {
    auto harness = std::make_shared<HandlerHarness>(delay);
    harnesses_.push_back(harness);
    return harness;
  }

  // Closes every handler, even after a test stopped early; a held run is released first,
  // since close() waits for the running callback to return.
  void TearDown() override
  {
    for (const auto & harness : harnesses_) {
      harness->callback.releaseFirstRun();
      EXPECT_TRUE(closesInTime(harness)) << "close() did not return.";
    }
  }

  static bool closesInTime(const std::shared_ptr<HandlerHarness> & harness)
  {
    return test_support::finishesWithin(kHangTimeout, [harness]() { harness->handler.close(); });
  }

private:
  std::vector<std::shared_ptr<HandlerHarness>> harnesses_;
};

TEST_F(PipelineFailureHandlerTest, ScheduleRunsCallbackOnceAndCoalesces)
{
  const auto harness = makeHarness(std::chrono::milliseconds(30));

  EXPECT_TRUE(harness->handler.schedule());
  // The second schedule lands while the first is still pending; it must be
  // folded into that one callback, not queue another.
  EXPECT_FALSE(harness->handler.schedule());
  EXPECT_TRUE(harness->callback.waitForRunCount(1U));

  // Give a coalesced duplicate a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(harness->callback.runCount(), 1U);
}

TEST_F(PipelineFailureHandlerTest, CloseDropsADelayedRunAndRejectsRequests)
{
  const auto harness = makeHarness(std::chrono::seconds(10));

  EXPECT_TRUE(harness->handler.schedule());
  // close() wakes the worker out of the delay instead of waiting it out.
  ASSERT_TRUE(closesInTime(harness)) << "close() did not return.";

  EXPECT_EQ(harness->callback.runCount(), 0U);
  EXPECT_FALSE(harness->handler.schedule());
  EXPECT_FALSE(harness->handler.requestRun());
}

TEST_F(PipelineFailureHandlerTest, CancelPendingPreventsCallback)
{
  const auto harness = makeHarness(std::chrono::milliseconds(200));

  EXPECT_TRUE(harness->handler.schedule());
  harness->handler.cancelPending();

  // Wait past the scheduled delay: a cancelled schedule never fires.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_EQ(harness->callback.runCount(), 0U);

  // Nothing is left pending, so the next failure schedules a run again.
  EXPECT_TRUE(harness->handler.schedule());
  EXPECT_TRUE(harness->callback.waitForRunCount(1U));
}

// AudioOutputSink relies on this: a failure while its restart runs must not restart it again.
TEST_F(PipelineFailureHandlerTest, ScheduleFromAnotherThreadWhileCallbackRunsIsIgnored)
{
  const auto harness = makeHarness(std::chrono::milliseconds(20));
  harness->callback.holdFirstRun();

  EXPECT_TRUE(harness->handler.schedule());
  ASSERT_TRUE(harness->callback.waitForRunCount(1U));

  EXPECT_FALSE(harness->handler.schedule());
  harness->callback.releaseFirstRun();

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(harness->callback.runCount(), 1U);

  // Once the callback has returned, a new failure schedules a run again.
  EXPECT_TRUE(harness->handler.schedule());
  EXPECT_TRUE(harness->callback.waitForRunCount(2U));
}

// RosStream uses no delay, and its run can report a failure from inside the callback on the
// same thread; schedule() must refuse it so the run does not restart itself.
TEST_F(PipelineFailureHandlerTest, ScheduleFromInsideCallbackWithoutDelayIsIgnored)
{
  const auto harness = makeHarness(std::chrono::milliseconds::zero());
  const auto results_during_run = std::make_shared<std::vector<bool>>();
  harness->callback.setFirstRunAction(
    [&handler = harness->handler, results_during_run]() { results_during_run->push_back(handler.schedule()); });

  EXPECT_TRUE(harness->handler.schedule());
  ASSERT_TRUE(harness->callback.waitForRunCount(1U));

  // Give a second run a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  ASSERT_TRUE(closesInTime(harness)) << "close() did not return.";

  // close() joined the worker, so the callback's writes are visible here.
  EXPECT_EQ(*results_during_run, (std::vector<bool>{false}));
  EXPECT_EQ(harness->callback.runCount(), 1U);
}

// The destructor closes the handler like close() does, so it waits for a running callback.
TEST_F(PipelineFailureHandlerTest, DestroyingAnOpenHandlerJoinsItsWorker)
{
  auto harness = std::make_shared<HandlerHarness>(std::chrono::milliseconds(20));
  const auto callback_returned = std::make_shared<std::atomic<bool>>(false);
  harness->callback.setFirstRunAction([callback_returned]() {
    // Keeps the callback running while the handler is destroyed.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    callback_returned->store(true);
  });

  EXPECT_TRUE(harness->handler.schedule());
  ASSERT_TRUE(harness->callback.waitForRunCount(1U));
  // The task takes the only reference, so the handler is destroyed on the task's thread.
  const bool destroyed_in_time = test_support::finishesWithin(
    kHangTimeout, [owned_harness = std::move(harness)]() mutable { owned_harness.reset(); });
  ASSERT_TRUE(destroyed_in_time) << "The destructor did not return.";

  EXPECT_TRUE(callback_returned->load());
}

// The return value tells the caller whether to log: true for a new run or a rerun, false when
// the request joins one already asked for. A rerun waits the usual delay like any other run.
TEST_F(PipelineFailureHandlerTest, RequestRunReportsWhetherItAskedForANewRun)
{
  constexpr auto kDelay = std::chrono::milliseconds(100);
  const auto harness = makeHarness(kDelay);
  const auto results_during_run = std::make_shared<std::vector<bool>>();
  harness->callback.setFirstRunAction([&handler = harness->handler, results_during_run]() {
    results_during_run->push_back(handler.requestRun());
    results_during_run->push_back(handler.requestRun());
  });

  EXPECT_TRUE(harness->handler.requestRun());
  EXPECT_FALSE(harness->handler.requestRun());
  ASSERT_TRUE(harness->callback.waitForRunCount(2U));

  // Give an unexpected third run a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  ASSERT_TRUE(closesInTime(harness)) << "close() did not return.";

  // close() joined the worker, so the callback's writes are visible here.
  EXPECT_EQ(*results_during_run, (std::vector<bool>{true, false}));
  const auto run_times = harness->callback.runTimes();
  ASSERT_EQ(run_times.size(), 2U);
  EXPECT_GE(run_times[1] - run_times[0], kDelay);
}

// A stream's failure can land, from another thread, just as its restart returns. Whichever
// side of the return requestRun() lands on, it must ask for and lead to another run. At real
// timings this cannot tell one locked call from two; it pins the outcome on both sides.
TEST_F(PipelineFailureHandlerTest, RequestRunRacingTheCallbackReturnLeadsToAnotherRun)
{
  for (int attempt = 0; attempt < 20; ++attempt) {
    const auto harness = makeHarness(std::chrono::milliseconds(1));
    harness->callback.holdFirstRun();

    ASSERT_TRUE(harness->handler.schedule());
    ASSERT_TRUE(harness->callback.waitForRunCount(1U));
    const auto requested = std::make_shared<bool>(false);
    const auto request_as_callback_returns = [harness, requested]() {
      std::thread releaser([harness]() { harness->callback.releaseFirstRun(); });
      *requested = harness->handler.requestRun();
      releaser.join();
    };
    ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, request_as_callback_returns))
      << "requestRun() did not return; attempt " << attempt;

    EXPECT_TRUE(*requested) << "attempt " << attempt;
    EXPECT_TRUE(harness->callback.waitForRunCount(2U)) << "attempt " << attempt;
    ASSERT_TRUE(closesInTime(harness)) << "close() did not return; attempt " << attempt;
  }
}

// A stream's restart cancels failures that the pipeline it stopped reported during the run.
TEST_F(PipelineFailureHandlerTest, CancelPendingWhileCallbackRunsDropsRequestedRerun)
{
  const auto harness = makeHarness(std::chrono::milliseconds(20));
  harness->callback.setFirstRunAction([&handler = harness->handler]() {
    handler.requestRun();
    handler.cancelPending();
  });

  EXPECT_TRUE(harness->handler.schedule());
  ASSERT_TRUE(harness->callback.waitForRunCount(1U));

  // Give a rerun that was not dropped a chance to surface before declaring success.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(harness->callback.runCount(), 1U);
}

TEST_F(PipelineFailureHandlerTest, CloseWhileCallbackRunsDropsRequestedRerun)
{
  // The rerun waits out the delay after the running callback returns, so close()
  // has that long to land even if it starts late; either way the requested rerun
  // must be dropped and close() must return.
  const auto harness = makeHarness(std::chrono::milliseconds(500));
  harness->callback.holdFirstRun();

  EXPECT_TRUE(harness->handler.schedule());
  ASSERT_TRUE(harness->callback.waitForRunCount(1U));
  EXPECT_TRUE(harness->handler.requestRun());

  const auto close_while_callback_runs = [harness]() {
    // Usually lets close() mark the handler closed while the callback still runs.
    std::thread releaser([harness]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      harness->callback.releaseFirstRun();
    });
    harness->handler.close();
    releaser.join();
  };
  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, close_while_callback_runs)) << "close() did not return.";

  EXPECT_EQ(harness->callback.runCount(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge::utils
