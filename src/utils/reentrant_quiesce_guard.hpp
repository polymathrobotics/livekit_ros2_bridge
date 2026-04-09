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
#include <mutex>
#include <stdexcept>
#include <thread>

namespace livekit_ros2_bridge
{

// Coordinates shutdown of a single active callback/drain section. External callers wait until the
// section goes inactive, while the owning thread can quiesce itself without self-deadlocking.
class ReentrantQuiesceGuard
{
public:
  ReentrantQuiesceGuard() = default;

  ReentrantQuiesceGuard(const ReentrantQuiesceGuard &) = delete;
  ReentrantQuiesceGuard & operator=(const ReentrantQuiesceGuard &) = delete;
  ReentrantQuiesceGuard(ReentrantQuiesceGuard &&) = delete;
  ReentrantQuiesceGuard & operator=(ReentrantQuiesceGuard &&) = delete;

  // Starts work only while entry is enabled and no other thread currently owns the guarded
  // section.
  bool tryBeginWork()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || active_) {
      return false;
    }

    active_ = true;
    owner_thread_id_ = std::this_thread::get_id();
    return true;
  }

  // Starts work when the caller expects the guarded section to be enabled and currently idle.
  void beginWork()
  {
    if (!tryBeginWork()) {
      throw std::logic_error("ReentrantQuiesceGuard work must begin from an enabled, idle state.");
    }
  }

  // Finishes the active section and wakes any thread blocked in quiesce().
  void endWork()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        return;
      }

      active_ = false;
      owner_thread_id_ = std::thread::id{};
    }

    quiesced_.notify_all();
  }

  // Prevents future tryBeginWork() calls from entering the guarded section.
  void disable()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
  }

  // Waits until the section is inactive, or until the caller is itself the current owner.
  void quiesce()
  {
    const auto caller_thread_id = std::this_thread::get_id();
    std::unique_lock<std::mutex> lock(mutex_);
    quiesced_.wait(lock, [this, &caller_thread_id]() { return !active_ || owner_thread_id_ == caller_thread_id; });
  }

private:
  std::mutex mutex_;
  std::condition_variable quiesced_;
  bool enabled_ = true;
  bool active_ = false;
  std::thread::id owner_thread_id_;
};

}  // namespace livekit_ros2_bridge
