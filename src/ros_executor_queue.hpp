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

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/waitable.hpp"

namespace livekit_ros2_bridge
{

class ExecutorWakeWaitable;

class RosExecutorQueue final
{
public:
  explicit RosExecutorQueue(rclcpp::Node & node);
  ~RosExecutorQueue();

  template <typename Fn>
  auto submit(Fn && fn) -> std::future<std::invoke_result_t<Fn>>
  {
    using Result = std::invoke_result_t<Fn>;

    auto promise = std::make_shared<std::promise<Result>>();
    auto future = promise->get_future();

    PendingTask task;
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

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) {
        task.cancel();
        return future;
      }
      tasks_.push(std::move(task));
    }

    wake_executor();

    return future;
  }

  void shutdown();

private:
  struct PendingTask
  {
    std::function<void()> run;
    std::function<void()> cancel;
  };

  void drain();
  void wake_executor();

  std::mutex mutex_;
  std::condition_variable drain_finished_;
  std::queue<PendingTask> tasks_;
  std::shared_ptr<ExecutorWakeWaitable> waitable_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_interface_;
  bool shutdown_ = false;
  bool drain_active_ = false;
  std::thread::id drain_thread_id_;
};

}  // namespace livekit_ros2_bridge
