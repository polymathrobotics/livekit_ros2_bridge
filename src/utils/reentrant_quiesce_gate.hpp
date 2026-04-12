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

const auto kNoOwningThread = std::thread::id{};

// Coordinates shutdown of a single callback/drain entry gate. External callers wait until the
// active entry leaves, while the owning thread can await idle without self-deadlocking.
class ReentrantQuiesceGate
{
public:
  ReentrantQuiesceGate() = default;

  ReentrantQuiesceGate(const ReentrantQuiesceGate &) = delete;
  ReentrantQuiesceGate & operator=(const ReentrantQuiesceGate &) = delete;
  ReentrantQuiesceGate(ReentrantQuiesceGate &&) = delete;
  ReentrantQuiesceGate & operator=(ReentrantQuiesceGate &&) = delete;

  // Admits entry only while the gate is open and no other thread currently owns the guarded
  // section.
  bool tryEnter()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_open_ || is_entered_) {
      return false;
    }

    is_entered_ = true;
    owner_thread_id_ = std::this_thread::get_id();
    return true;
  }

  // Enters the guarded section when the caller expects the gate to be open and currently idle.
  void enter()
  {
    if (!tryEnter()) {
      throw std::logic_error("ReentrantQuiesceGate entry must begin from an open, idle state.");
    }
  }

  // Leaves the active section and wakes any thread blocked in awaitIdle().
  void leave()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!is_entered_) {
        return;
      }

      is_entered_ = false;
      owner_thread_id_ = kNoOwningThread;
    }

    idle_.notify_all();
  }

  // Closes the gate so future tryEnter() calls cannot enter the guarded section.
  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_open_ = false;
  }

  // Waits until the section is idle, or until the caller is itself the current owner.
  void awaitIdle()
  {
    const auto caller_thread_id = std::this_thread::get_id();
    std::unique_lock<std::mutex> lock(mutex_);
    idle_.wait(lock, [this, &caller_thread_id]() {
      const bool caller_owns_active_section = owner_thread_id_ == caller_thread_id;
      return !is_entered_ || caller_owns_active_section;
    });
  }

private:
  std::mutex mutex_;
  std::condition_variable idle_;
  bool is_open_ = true;
  bool is_entered_ = false;
  std::thread::id owner_thread_id_{kNoOwningThread};
};

}  // namespace livekit_ros2_bridge
