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
// A failure scheduled while the callback runs (e.g. the restarted pipeline fails
// at once) is kept and runs the callback once more, after the same delay.
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
  // each recovery once. While the callback runs, the first call schedules the
  // follow-up run and returns true; later calls fold into it and return false.
  bool schedule();
  // Drops a scheduled run that has not started, including a follow-up run
  // requested while the callback runs.
  void cancelPending();
  void close();

private:
  void run();

  std::chrono::milliseconds delay_;
  Callback callback_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool closed_ = false;
  // Set by schedule(), cleared when the callback starts, so a schedule() during
  // the callback queues the next run.
  bool pending_ = false;
  std::thread worker_;
};

}  // namespace livekit_ros2_bridge::utils
