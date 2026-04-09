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

#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>

namespace livekit_ros2_bridge
{

// Coordinates callback shutdown/reset by blocking new work, draining in-flight work, and
// advancing a generation so queued callbacks from the old session can self-reject on entry.
class QuiesceGuard
{
public:
  QuiesceGuard() = default;

  QuiesceGuard(const QuiesceGuard &) = delete;
  QuiesceGuard & operator=(const QuiesceGuard &) = delete;
  QuiesceGuard(QuiesceGuard &&) = delete;
  QuiesceGuard & operator=(QuiesceGuard &&) = delete;

  // Callers capture this generation alongside queued work and must present the same generation
  // back to tryBeginWork() before touching shared state.
  std::size_t currentGeneration() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  // Starts one unit of work only when callback entry is enabled and the caller still matches the
  // current generation.
  bool tryBeginWork(std::size_t expected_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || expected_generation != generation_) {
      return false;
    }

    ++active_count_;
    return true;
  }

  // Finishes one active work item and wakes any thread blocked in quiesce().
  void endWork()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_count_ == 0U) {
        return;
      }
      --active_count_;
    }

    quiesced_.notify_all();
  }

  // Stops new work from entering, advances the generation seen by queued callbacks, and waits
  // until every already-active work item has called endWork().
  std::size_t quiesce()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    enabled_ = false;
    ++generation_;
    quiesced_.wait(lock, [this]() { return active_count_ == 0U; });
    return generation_;
  }

  // Re-enables callback entry only when the caller still refers to the current generation.
  void resume(std::size_t generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_ != generation) {
      return;
    }

    enabled_ = true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable quiesced_;
  bool enabled_ = true;
  std::size_t active_count_ = 0U;
  std::size_t generation_ = 0U;
};

}  // namespace livekit_ros2_bridge
