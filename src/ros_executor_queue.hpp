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
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/waitable.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

using RosExecutorQueueNodeInterfaces = rclcpp::node_interfaces::
  NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeWaitablesInterface>;

// Runs submitted work from the node's ROS executor via a default-group waitable.
// Pending futures fail on shutdown; the node must outlive the queue.
class RosExecutorQueue final
{
public:
  RosExecutorQueue(RosExecutorQueueNodeInterfaces interfaces, rclcpp::Clock::SharedPtr clock);
  ~RosExecutorQueue();

  // Enqueues work in FIFO order. If shutdown() wins before drain() claims the
  // task, the returned future completes with the queue's shutdown error.
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

  // Cancels queued work and waits for any active drain from another thread.
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

  std::mutex mutex_;
  std::queue<Task> tasks_;

  // Cleared during shutdown so concurrent wake() becomes a no-op.
  std::shared_ptr<DrainWaitable> waitable_;
  // Retained so shutdown() can unregister waitable_ from the same node.
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_;

  rclcpp::Clock::SharedPtr log_clock_;

  bool shutdown_ = false;

  // Identifies the active drain so shutdown() can wait without deadlocking when
  // called by the drain owner.
  std::condition_variable drain_idle_;
  bool drain_active_ = false;
  std::thread::id drain_owner_thread_id_{};

  void drain();
  void wake();
  bool tryBeginDrain();
  void finishDrain();
  void awaitDrainIdle();
};

}  // namespace livekit_ros2_bridge
