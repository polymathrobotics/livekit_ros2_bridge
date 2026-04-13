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
constexpr int kReadyEntityId = 0;
}  // namespace

// Waitable adapter that turns the queue into a guard-condition "doorbell" for
// the executor. No per-wake payload flows through take_data(); execute() just
// hands control back to RosExecutorQueue::drain() to consume shared queue state.
class RosExecutorQueue::DrainWaitable final : public rclcpp::Waitable
{
public:
  DrainWaitable(RosExecutorQueue & queue, const rclcpp::Context::SharedPtr & context)
  : queue_(queue)
  , guard_condition_(std::make_shared<rclcpp::GuardCondition>(context))
  {}

#if RCLCPP_VERSION_GTE(28, 0, 0)
  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    guard_condition_->add_to_wait_set(wait_set);
  }

  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    const auto * rcl_guard_condition = &guard_condition_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      if (wait_set.guard_conditions[i] == rcl_guard_condition) {
        return true;
      }
    }
    return false;
  }

  void execute(const std::shared_ptr<void> & ignored_data) override
  {
    (void)ignored_data;
    queue_.drain();
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
    if (wait_set == nullptr) {
      return false;
    }

    const auto * rcl_guard_condition = &guard_condition_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set->size_of_guard_conditions; ++i) {
      if (wait_set->guard_conditions[i] == rcl_guard_condition) {
        return true;
      }
    }
    return false;
  }

  void execute(std::shared_ptr<void> & ignored_data) override
  {
    (void)ignored_data;
    queue_.drain();
  }
#endif

  std::shared_ptr<void> take_data() override
  {
    return nullptr;
  }

  std::shared_ptr<void> take_data_by_entity_id(size_t entity_id) override
  {
    (void)entity_id;
    return nullptr;
  }

  size_t get_number_of_ready_guard_conditions() override
  {
    return 1U;
  }

  void set_on_ready_callback(std::function<void(size_t, int)> on_ready) override
  {
    std::function<void(size_t, int)> ready_handler;
    size_t pending_wakes = 0U;

    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      on_ready_ = std::move(on_ready);
      if (on_ready_ == nullptr) {
        return;
      }

      ready_handler = on_ready_;
      pending_wakes = std::exchange(pending_wakes_, 0U);
    }

    // Some executors install the ready callback after wake() observes queued
    // work. Replay remembered wakes so the first pending task is not stranded.
    if (pending_wakes > 0U) {
      ready_handler(pending_wakes, kReadyEntityId);
    }
  }

  void clear_on_ready_callback() override
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    on_ready_ = nullptr;
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const
  {
    return {};
  }

  void wake()
  {
    // If the executor has not installed a ready callback yet, remember this
    // wake so set_on_ready_callback() can replay it after registration.
    guard_condition_->trigger();
    std::function<void(size_t, int)> ready_handler;

    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (on_ready_ == nullptr) {
        ++pending_wakes_;
        return;
      }

      ready_handler = on_ready_;
    }

    ready_handler(1U, kReadyEntityId);
  }

private:
  RosExecutorQueue & queue_;
  rclcpp::GuardCondition::SharedPtr guard_condition_;
  mutable std::mutex callback_mutex_;
  std::function<void(size_t, int)> on_ready_;
  size_t pending_wakes_ = 0U;
};

RosExecutorQueue::RosExecutorQueue(rclcpp::Node & node)
: default_callback_group_(node.get_node_base_interface()->get_default_callback_group())
, waitables_(node.get_node_waitables_interface())
, logger_(kRosExecutorQueueLogger)
, log_clock_(node.get_clock())
{
  waitable_ = std::make_shared<DrainWaitable>(*this, node.get_node_base_interface()->get_context());
  waitables_->add_waitable(waitable_, default_callback_group_);
}

RosExecutorQueue::~RosExecutorQueue()
{
  shutdown();
}

void RosExecutorQueue::wake()
{
  std::shared_ptr<DrainWaitable> waitable;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    waitable = waitable_;
  }

  if (waitable == nullptr) {
    return;
  }

  try {
    waitable->wake();
  } catch (const std::exception & exc) {
    LogEvent(kRosExecutorQueueLogger, "executor_wake_failed")
      .field("action", "shutdown")
      .field("error", exc.what())
      .error();
    shutdown();
  } catch (...) {
    LogEvent(kRosExecutorQueueLogger, "executor_wake_failed")
      .field("action", "shutdown")
      .field("error", "unknown_exception")
      .error();
    shutdown();
  }
}

void RosExecutorQueue::shutdown()
{
  // Only the thread that flips shutdown_ tears down the waitable and cancels
  // queued work. Concurrent shutdown callers just wait for drain() to go idle.
  std::queue<Task> pending_tasks;
  std::shared_ptr<DrainWaitable> waitable;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
  rclcpp::CallbackGroup::SharedPtr default_callback_group;
  bool already_shutdown = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) {
      already_shutdown = true;
    } else {
      shutdown_ = true;
      pending_tasks = std::move(tasks_);
      waitable = std::move(waitable_);
      waitables = std::move(waitables_);
      default_callback_group = std::move(default_callback_group_);
    }
  }

  if (already_shutdown) {
    drain_gate_.awaitIdle();
    return;
  }

  const std::size_t canceled_count = pending_tasks.size();
  drain_gate_.close();

  if (waitable != nullptr && waitables != nullptr && default_callback_group != nullptr) {
    waitables->remove_waitable(waitable, default_callback_group);
  }

  if (canceled_count > 0U) {
    LogEvent(kRosExecutorQueueLogger, "executor_pending_tasks_canceled")
      .field("reason", "shutdown")
      .field("count", canceled_count)
      .warn();
  }

  while (!pending_tasks.empty()) {
    Task task = std::move(pending_tasks.front());
    pending_tasks.pop();
    task.cancel();
  }

  // Already-started drain work runs to completion; only tasks still in the
  // moved-out task snapshot above are canceled during shutdown.
  drain_gate_.awaitIdle();
}

void RosExecutorQueue::drain()
{
  if (!drain_gate_.tryEnter()) {
    return;
  }
  ScopeExit finish_drain([this]() { drain_gate_.leave(); });

  // Keep draining until the queue is empty so tasks submitted from active
  // queue work are consumed by the same executor wakeup.
  while (true) {
    Task task;

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
        .field("action", "continue")
        .field("error", exc.what())
        .error();
    } catch (...) {
      LogEvent(kRosExecutorQueueLogger, "executor_task_failed")
        .field("action", "continue")
        .field("error", "unknown_exception")
        .error();
    }
  }
}

}  // namespace livekit_ros2_bridge
