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

#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/waitable.hpp"
#include "ros_node_interfaces.hpp"
#include "utils/log_event.hpp"
#include "utils/reentrant_quiesce_gate.hpp"

namespace livekit_ros2_bridge
{

// Queues work that must run on the node's ROS executor thread. Work is exposed
// through a waitable on the node's default callback group, so submitted futures
// either complete from that executor context or fail with a shutdown error if
// the task never starts draining. The queue borrows the node's waitables and
// callback-group interfaces, so the node must outlive this queue.
class RosExecutorQueue final
{
public:
  explicit RosExecutorQueue(rclcpp::Node & node);
  explicit RosExecutorQueue(ExecutorNodeInterfaces interfaces);
  ~RosExecutorQueue();

  // Enqueues work in FIFO order for execution on the executor thread. If
  // shutdown() wins the race before this task begins draining, the returned
  // future completes with the queue's shutdown error instead.
  template <typename Fn>
  auto submit(Fn && fn) -> std::future<std::invoke_result_t<Fn>>
  {
    using Result = std::invoke_result_t<Fn>;

    auto promise = std::make_shared<std::promise<Result>>();
    auto future = promise->get_future();

    Task task;
    task.run = [promise, fn = std::forward<Fn>(fn)]() mutable {
      try {
        if constexpr (std::is_void_v<Result>) {
          fn();
          promise->set_value();
        } else {
          promise->set_value(fn());
        }
      } catch (...) {
        promise->set_exception(std::current_exception());
      }
    };
    task.cancel = [promise]() {
      try {
        throw std::runtime_error("ROS executor queue is shut down.");
      } catch (...) {
        promise->set_exception(std::current_exception());
      }
    };

    std::unique_lock<std::mutex> lock(mutex_);
    if (shutdown_) {
      lock.unlock();
      LogEvent(logger_, "executor_task_rejected").field("reason", "shutdown").warnThrottle(*log_clock_, kLogThrottle);
      task.cancel();
      return future;
    }

    tasks_.push(std::move(task));
    lock.unlock();

    wake();

    return future;
  }

  // Cancels queued-but-not-yet-started work and waits for any other thread
  // already draining the queue to leave that critical section.
  void shutdown();

private:
  class DrainWaitable;

  struct Task
  {
    std::function<void()> run;
    std::function<void()> cancel;
  };

  static constexpr auto kLogThrottle = std::chrono::seconds(5);
  inline static const auto logger_ = rclcpp::get_logger("ros_executor_queue");

  // Protects shutdown_ and all state shared between submit(), wake(), drain(),
  // and shutdown().
  std::mutex mutex_;
  std::queue<Task> tasks_;

  // Cleared during shutdown() before the waitable is detached so concurrent
  // wake() callers either use the live waitable or cleanly become a no-op.
  std::shared_ptr<DrainWaitable> waitable_;
  // Retained solely so shutdown() can unregister waitable_ from the same
  // node interfaces it was added to.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_;

  rclcpp::Clock::SharedPtr log_clock_;

  // Once set, new submissions are rejected and only work already running in
  // drain() is allowed to finish.
  bool shutdown_ = false;

  // Used so shutdown() can wait for an in-progress drain without deadlocking
  // when shutdown itself is called from the executor thread running drain().
  ReentrantQuiesceGate drain_gate_;

  void drain();
  void wake();
};

}  // namespace livekit_ros2_bridge
