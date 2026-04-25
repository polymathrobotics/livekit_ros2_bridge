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
#include "rclcpp/version.h"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge
{

namespace
{
constexpr int kReadyEntityId = 0;
}  // namespace

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
    guard_condition_->add_to_wait_set(wait_set);
  }

  bool is_ready(rcl_wait_set_t * wait_set) override
  {
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
    if (on_ready == nullptr) {
      clear_on_ready_callback();
      return;
    }

    guard_condition_->set_on_trigger_callback(
      [on_ready = std::move(on_ready)](size_t wake_count) { on_ready(wake_count, kReadyEntityId); });
  }

  void clear_on_ready_callback() override
  {
    guard_condition_->set_on_trigger_callback(nullptr);
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const
  {
    return {};
  }

  void wake()
  {
    guard_condition_->trigger();
  }

private:
  RosExecutorQueue & queue_;
  rclcpp::GuardCondition::SharedPtr guard_condition_;
};

RosExecutorQueue::RosExecutorQueue(RosExecutorQueueNodeInterfaces interfaces, rclcpp::Clock::SharedPtr clock)
: waitables_(interfaces.get_node_waitables_interface())
, log_clock_(std::move(clock))
{
  const auto base = interfaces.get_node_base_interface();
  waitable_ = std::make_shared<DrainWaitable>(*this, base->get_context());
  waitables_->add_waitable(waitable_, nullptr);
}

RosExecutorQueue::~RosExecutorQueue()
{
  shutdown();
}

void RosExecutorQueue::shutdown()
{
  // The thread that flips shutdown_ owns waitable teardown and queued task
  // cancellation. Concurrent shutdown callers only wait for drain() to go idle.
  std::queue<Task> queued_tasks;
  std::shared_ptr<DrainWaitable> waitable;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;

  std::unique_lock<std::mutex> lock(mutex_);
  if (shutdown_) {
    lock.unlock();
    awaitDrainIdle();
    return;
  }

  shutdown_ = true;
  queued_tasks = std::move(tasks_);
  waitable = std::move(waitable_);
  waitables = std::move(waitables_);
  lock.unlock();

  const std::size_t canceled_count = queued_tasks.size();

  if (waitable != nullptr && waitables != nullptr) {
    waitables->remove_waitable(waitable, nullptr);
  }

  if (canceled_count > 0U) {
    LogEvent(logger_, "executor_pending_tasks_canceled")
      .field("reason", "shutdown")
      .field("count", canceled_count)
      .warn();
  }

  while (!queued_tasks.empty()) {
    Task task = std::move(queued_tasks.front());
    queued_tasks.pop();
    task.cancel();
  }

  // Already-started drain work runs to completion; only tasks still in the
  // moved-out task snapshot above are canceled during shutdown.
  awaitDrainIdle();
}

void RosExecutorQueue::drain()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || drain_active_) {
      return;
    }

    drain_active_ = true;
    drain_owner_thread_id_ = std::this_thread::get_id();
  }
  ScopeExit finish_drain([this]() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      drain_active_ = false;
      drain_owner_thread_id_ = std::thread::id{};
    }

    drain_idle_.notify_all();
  });

  // Tasks submitted from active queue work are consumed by the same wakeup.
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
      // Queued work keeps the callback-group affinity of the consuming executor thread.
      task.run();
    } catch (...) {
      LogEvent(logger_, "executor_task_failed")
        .field("action", "continue")
        .fieldException("error", std::current_exception())
        .error();
    }
  }
}

void RosExecutorQueue::awaitDrainIdle()
{
  const auto caller_thread_id = std::this_thread::get_id();
  std::unique_lock<std::mutex> lock(mutex_);
  // shutdown() can be called from queued work; the drain owner must not wait
  // for itself to leave drain().
  drain_idle_.wait(
    lock, [this, caller_thread_id]() { return !drain_active_ || drain_owner_thread_id_ == caller_thread_id; });
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
  } catch (...) {
    LogEvent(logger_, "executor_wake_failed")
      .field("action", "shutdown")
      .fieldException("error", std::current_exception())
      .error();
    shutdown();
  }
}

}  // namespace livekit_ros2_bridge
