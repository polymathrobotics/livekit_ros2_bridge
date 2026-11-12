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
#include <exception>
#include <future>
#include <thread>
#include <utility>

namespace livekit_ros2_bridge::test_support
{

// Runs `task` on a detached thread and reports whether it finished within `timeout`, so a
// deadlock fails the test instead of hanging the binary. A stuck task is abandoned, never
// joined: it must own (shared_ptr) everything it touches. Its exceptions are rethrown here.
template <typename Task>
bool finishesWithin(std::chrono::milliseconds timeout, Task task)
{
  std::promise<void> finished;
  std::future<void> finished_future = finished.get_future();
  std::thread([task = std::move(task), finished = std::move(finished)]() mutable {
    std::exception_ptr failure;
    {
      // Moved out so the task's state is released before the caller is told it finished.
      Task owned_task = std::move(task);
      try {
        owned_task();
      } catch (...) {
        failure = std::current_exception();
      }
    }
    if (failure != nullptr) {
      finished.set_exception(failure);
      return;
    }
    finished.set_value();
  }).detach();

  if (finished_future.wait_for(timeout) != std::future_status::ready) {
    return false;
  }
  finished_future.get();
  return true;
}

}  // namespace livekit_ros2_bridge::test_support
