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
constexpr int kGuardEntityId = 0;
}  // namespace

class RosExecutorQueue::DrainWaitable final : public rclcpp::Waitable
{
public:
  DrainWaitable(RosExecutorQueue & queue, const rclcpp::Context::SharedPtr & context)
  : queue_(queue)
  , guard_(std::make_shared<rclcpp::GuardCondition>(context))
  {}

#if RCLCPP_VERSION_GTE(28, 0, 0)
  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    guard_->add_to_wait_set(wait_set);
  }

  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    const auto * condition = &guard_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      if (wait_set.guard_conditions[i] == condition) {
        return true;
      }
    }
    return false;
  }

  void execute(const std::shared_ptr<void> & data) override
  {
    (void)data;
    queue_.drain();
  }
#else
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    guard_->add_to_wait_set(wait_set);
  }

  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    const auto * condition = &guard_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set->size_of_guard_conditions; ++i) {
      if (wait_set->guard_conditions[i] == condition) {
        return true;
      }
    }
    return false;
  }

  void execute(std::shared_ptr<void> & data) override
  {
    (void)data;
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

    guard_->set_on_trigger_callback(
      [on_ready = std::move(on_ready)](size_t count) { on_ready(count, kGuardEntityId); });
  }

  void clear_on_ready_callback() override
  {
    guard_->set_on_trigger_callback(nullptr);
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const
  {
    return {};
  }

  void wake()
  {
    guard_->trigger();
  }

private:
  RosExecutorQueue & queue_;
  rclcpp::GuardCondition::SharedPtr guard_;
};

RosExecutorQueue::RosExecutorQueue(RosExecutorQueue::NodeInterfaces interfaces, rclcpp::Clock::SharedPtr clock)
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
  std::queue<Task> queued;
  std::shared_ptr<DrainWaitable> waitable;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;

  std::unique_lock<std::mutex> lock(mutex_);
  if (shutdown_) {
    lock.unlock();
    awaitIdle();
    return;
  }

  shutdown_ = true;
  queued = std::move(tasks_);
  waitable = std::move(waitable_);
  waitables = std::move(waitables_);
  lock.unlock();

  const std::size_t canceled = queued.size();

  if (waitable != nullptr && waitables != nullptr) {
    waitables->remove_waitable(waitable, nullptr);
  }

  if (canceled > 0U) {
    LogEvent(logger_, "executor_pending_tasks_canceled").field("count", canceled).warn();
  }

  while (!queued.empty()) {
    Task task = std::move(queued.front());
    queued.pop();
    task.cancel();
  }

  // Already-started drain work runs to completion; only tasks still in the
  // moved-out task snapshot above are canceled during shutdown.
  awaitIdle();
}

void RosExecutorQueue::drain()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || draining_) {
      return;
    }

    draining_ = true;
    drain_owner_ = std::this_thread::get_id();
  }
  ScopeExit finish([this]() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      draining_ = false;
      drain_owner_ = std::thread::id{};
    }

    idle_.notify_all();
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
      LogEvent(logger_, "executor_task_failed").fieldException("error", std::current_exception()).error();
    }
  }
}

void RosExecutorQueue::awaitIdle()
{
  const auto caller = std::this_thread::get_id();
  std::unique_lock<std::mutex> lock(mutex_);
  // shutdown() can be called from queued work; the drain owner must not wait
  // for itself to leave drain().
  idle_.wait(lock, [this, caller]() { return !draining_ || drain_owner_ == caller; });
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
