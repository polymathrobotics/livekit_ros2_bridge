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
#include <thread>
#include <utility>

namespace livekit_ros2_bridge
{

class CallbackGate final
{
public:
  enum class Concurrency
  {
    Concurrent,
    Exclusive,
  };

  enum class WaitMode
  {
    AllCallbacks,
    ExcludingCurrentThread,
  };

  explicit CallbackGate(Concurrency concurrency = Concurrency::Concurrent)
  : concurrency_(concurrency)
  {}

  CallbackGate(const CallbackGate &) = delete;
  CallbackGate & operator=(const CallbackGate &) = delete;
  CallbackGate(CallbackGate &&) = delete;
  CallbackGate & operator=(CallbackGate &&) = delete;

  // Runs fn only while the gate is open. Returns false when close has started
  // or when exclusive mode already has active work.
  template <typename Fn>
  bool run(Fn && fn)
  {
    if (!tryEnter()) {
      return false;
    }

    struct Active
    {
      CallbackGate & gate;

      ~Active()
      {
        gate.leave();
      }
    } active{*this};

    std::forward<Fn>(fn)();
    return true;
  }

  // Closes the gate and waits for active callbacks. The return value is true
  // only for the caller that first closed the gate. ExcludingCurrentThread is
  // supported by exclusive gates, where there can be only one active owner.
  bool closeAndWait(WaitMode wait_mode = WaitMode::AllCallbacks)
  {
    const auto caller = wait_mode == WaitMode::ExcludingCurrentThread ? std::this_thread::get_id() : std::thread::id{};
    std::unique_lock<std::mutex> lock(mutex_);
    const bool first_close = !closed_;
    closed_ = true;
    idle_.wait(lock, [this, wait_mode, caller]() { return activeCallbacksToWaitFor(caller, wait_mode) == 0U; });
    return first_close;
  }

  bool isClosed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

private:
  bool tryEnter()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    if (concurrency_ == Concurrency::Exclusive && active_ != 0U) {
      return false;
    }

    ++active_;
    if (concurrency_ == Concurrency::Exclusive) {
      exclusive_owner_thread_id_ = std::this_thread::get_id();
    }
    return true;
  }

  void leave()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      --active_;

      if (concurrency_ == Concurrency::Exclusive) {
        const auto caller = std::this_thread::get_id();
        if (exclusive_owner_thread_id_ == caller) {
          exclusive_owner_thread_id_ = std::thread::id{};
        }
      }
    }

    idle_.notify_all();
  }

  std::size_t activeCallbacksToWaitFor(std::thread::id caller, WaitMode wait_mode) const
  {
    if (wait_mode == WaitMode::AllCallbacks) {
      return active_;
    }

    if (concurrency_ == Concurrency::Exclusive && active_ != 0U && exclusive_owner_thread_id_ == caller) {
      return 0U;
    }
    return active_;
  }

  const Concurrency concurrency_;
  mutable std::mutex mutex_;
  std::condition_variable idle_;
  bool closed_ = false;
  std::size_t active_ = 0U;
  std::thread::id exclusive_owner_thread_id_;
};

}  // namespace livekit_ros2_bridge
