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

#include "ros_executor_queue.hpp"

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/version.h"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge
{

namespace
{
const auto kRosExecutorQueueLogger = rclcpp::get_logger("ros_executor_queue");
constexpr int kGuardConditionReadyEntityId = 0;
}  // namespace

class ExecutorWakeWaitable final : public rclcpp::Waitable
{
public:
  ExecutorWakeWaitable(const rclcpp::Context::SharedPtr & context, std::function<void()> execute_callback)
  : guard_condition_(std::make_shared<rclcpp::GuardCondition>(context))
  , execute_callback_(std::move(execute_callback))
  {
    if (!execute_callback_) {
      throw std::invalid_argument("Executor wake callback must not be empty.");
    }
  }

#if RCLCPP_VERSION_GTE(28, 0, 0)
  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    guard_condition_->add_to_wait_set(wait_set);
  }

  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    return is_ready_impl(wait_set);
  }

  void execute(const std::shared_ptr<void> & data) override
  {
    (void)data;
    execute_impl();
  }
#else
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    if (wait_set == nullptr) {
      throw std::invalid_argument("wait set is null");
    }
    guard_condition_->add_to_wait_set(wait_set);
  }

  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    return wait_set != nullptr && is_ready_impl(*wait_set);
  }

  void execute(std::shared_ptr<void> & data) override
  {
    (void)data;
    execute_impl();
  }
#endif

  std::shared_ptr<void> take_data() override
  {
    return nullptr;
  }

  std::shared_ptr<void> take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return nullptr;
  }

  size_t get_number_of_ready_guard_conditions() override
  {
    return 1U;
  }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    std::function<void(size_t, int)> ready_callback;
    size_t unread_count = 0U;

    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      on_ready_callback_ = std::move(callback);
      ready_callback = on_ready_callback_;
      if (ready_callback) {
        unread_count = unread_count_;
        unread_count_ = 0U;
      }
    }

    if (unread_count > 0U && ready_callback) {
      ready_callback(unread_count, kGuardConditionReadyEntityId);
    }
  }

  void clear_on_ready_callback() override
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    on_ready_callback_ = nullptr;
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const
  {
    return {};
  }

  void wake_executor()
  {
    guard_condition_->trigger();

    std::function<void(size_t, int)> callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (on_ready_callback_) {
        callback = on_ready_callback_;
      } else {
        ++unread_count_;
      }
    }

    if (callback) {
      callback(1U, kGuardConditionReadyEntityId);
    }
  }

private:
  void execute_impl()
  {
    execute_callback_();
  }

  bool is_ready_impl(const rcl_wait_set_t & wait_set) const
  {
    const auto * target = &guard_condition_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      if (wait_set.guard_conditions[i] == target) {
        return true;
      }
    }
    return false;
  }

  rclcpp::GuardCondition::SharedPtr guard_condition_;
  std::function<void()> execute_callback_;
  mutable std::mutex callback_mutex_;
  std::function<void(size_t, int)> on_ready_callback_;
  size_t unread_count_ = 0U;
};

RosExecutorQueue::RosExecutorQueue(rclcpp::Node & node)
: callback_group_(node.get_node_base_interface()->get_default_callback_group())
, waitables_interface_(node.get_node_waitables_interface())
{
  waitable_ =
    std::make_shared<ExecutorWakeWaitable>(node.get_node_base_interface()->get_context(), [this]() { drain(); });
  waitables_interface_->add_waitable(waitable_, callback_group_);
}

RosExecutorQueue::~RosExecutorQueue()
{
  shutdown();
}

void RosExecutorQueue::wake_executor()
{
  std::shared_ptr<ExecutorWakeWaitable> waitable;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    waitable = waitable_;
  }

  if (waitable == nullptr) {
    return;
  }

  try {
    waitable->wake_executor();
  } catch (const std::exception & exc) {
    LogEvent(kRosExecutorQueueLogger, "executor_wake_failed")
      .field("reason", "exception")
      .field("action", "shutdown")
      .field("error", exc.what())
      .error();
    shutdown();
  } catch (...) {
    LogEvent(kRosExecutorQueueLogger, "executor_wake_failed")
      .field("reason", "unknown_error")
      .field("action", "shutdown")
      .error();
    shutdown();
  }
}

void RosExecutorQueue::shutdown()
{
  std::queue<PendingTask> pending;
  std::size_t canceled_count = 0U;
  std::shared_ptr<ExecutorWakeWaitable> waitable;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables_interface;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (shutdown_) {
      lock.unlock();
      drain_gate_.awaitIdle();
      return;
    }
    shutdown_ = true;
    pending = std::move(tasks_);
    canceled_count = pending.size();
    waitable = std::move(waitable_);
    waitables_interface = std::move(waitables_interface_);
    callback_group = std::move(callback_group_);
  }
  drain_gate_.close();

  if (waitables_interface && callback_group && waitable) {
    waitables_interface->remove_waitable(waitable, callback_group);
  }

  if (canceled_count > 0U) {
    LogEvent(kRosExecutorQueueLogger, "executor_queue_settled")
      .field("reason", "shutdown")
      .field("action", "cancel_pending")
      .field("count", canceled_count)
      .warn();
  }

  while (!pending.empty()) {
    PendingTask task = std::move(pending.front());
    pending.pop();
    task.cancel();
  }

  // Already-started drain work runs to completion; only tasks still in the
  // pending queue above are canceled during shutdown.
  drain_gate_.awaitIdle();
}

void RosExecutorQueue::drain()
{
  if (!drain_gate_.tryEnter()) {
    return;
  }
  ScopeExit finish_drain([this]() { drain_gate_.leave(); });

  while (true) {
    PendingTask task;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (tasks_.empty()) {
        return;
      }
      task = std::move(tasks_.front());
      tasks_.pop();
    }

    try {
      // drain() always runs on the executor thread that consumed the waitable,
      // so queued work observes the same callback-group affinity as ROS callbacks.
      task.run();
    } catch (const std::exception & exc) {
      LogEvent(kRosExecutorQueueLogger, "executor_task_failed")
        .field("reason", "exception")
        .field("action", "continue")
        .field("error", exc.what())
        .error();
    } catch (...) {
      LogEvent(kRosExecutorQueueLogger, "executor_task_failed")
        .field("reason", "unknown_exception")
        .field("action", "continue")
        .error();
    }
  }
}

}  // namespace livekit_ros2_bridge
