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

#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace livekit_ros2_bridge::utils
{

// Coalesces repeated GStreamer EOS/error callbacks into one stream-owned callback.
class PipelineFailureHandler final
{
public:
  using Callback = std::function<void()>;

  PipelineFailureHandler(std::chrono::milliseconds delay, Callback callback);
  ~PipelineFailureHandler();

  PipelineFailureHandler(const PipelineFailureHandler &) = delete;
  PipelineFailureHandler & operator=(const PipelineFailureHandler &) = delete;
  PipelineFailureHandler(PipelineFailureHandler &&) = delete;
  PipelineFailureHandler & operator=(PipelineFailureHandler &&) = delete;

  // Returns true only when this call newly schedules a run, so the caller logs
  // each recovery once. Returns false and does nothing while a run is pending,
  // while the callback runs (a failure posted by the restart itself is folded
  // into it), and once closed.
  bool schedule();
  // Asks for one more run after the usual delay, once the running callback
  // returns. Meant for the callback itself, e.g. after a restart that threw.
  // Repeated requests coalesce into one run. Does nothing when the callback is
  // not running or the handler is closed; close() and cancelPending() drop it.
  void rerunAfterCurrentRun();
  // Drops a rerun requested so far in the current run, e.g. by a failure from a
  // pipeline the callback has since stopped. A scheduled run is left alone.
  void discardRerunRequest();
  // Drops a scheduled run that has not started, including a requested rerun.
  void cancelPending();
  void close();

private:
  void run();

  std::chrono::milliseconds delay_;
  Callback callback_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool closed_ = false;
  bool pending_ = false;
  bool callback_running_ = false;
  // Set by rerunAfterCurrentRun() while the callback runs; becomes pending_ when it returns.
  bool rerun_requested_ = false;
  std::thread worker_;
};

}  // namespace livekit_ros2_bridge::utils
