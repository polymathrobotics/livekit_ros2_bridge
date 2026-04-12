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

// Coordinates callback shutdown/reset by closing callback entry, draining in-flight entries, and
// advancing a generation so queued callbacks from the old session can self-reject at the gate.
class QuiesceGate
{
public:
  QuiesceGate() = default;

  QuiesceGate(const QuiesceGate &) = delete;
  QuiesceGate & operator=(const QuiesceGate &) = delete;
  QuiesceGate(QuiesceGate &&) = delete;
  QuiesceGate & operator=(QuiesceGate &&) = delete;

  // Callers capture this generation alongside queued work and must present the same generation
  // back to tryEnter() before touching shared state.
  std::size_t currentGeneration() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  // Admits one callback entry only when the gate is open and the caller still matches the
  // current generation.
  bool tryEnter(std::size_t expected_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_open_ || expected_generation != generation_) {
      return false;
    }

    ++active_entries_;
    return true;
  }

  // Leaves one active callback entry and wakes any thread blocked in close().
  void leave()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_entries_ == 0U) {
        return;
      }
      --active_entries_;
    }

    idle_.notify_all();
  }

  // Closes callback entry, advances the generation seen by queued callbacks, and waits until
  // every already-admitted entry has called leave().
  std::size_t close()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    is_open_ = false;
    ++generation_;
    idle_.wait(lock, [this]() { return active_entries_ == 0U; });
    return generation_;
  }

  // Re-opens callback entry only when the caller still refers to the current generation.
  void open(std::size_t generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_ != generation) {
      return;
    }

    is_open_ = true;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable idle_;
  bool is_open_ = true;
  std::size_t active_entries_ = 0U;
  std::size_t generation_ = 0U;
};

}  // namespace livekit_ros2_bridge
