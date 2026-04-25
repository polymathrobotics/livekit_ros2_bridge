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

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros_executor_queue.hpp"
#include "ros_test_support.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename FutureT>
void expectRuntimeError(FutureT & future, const char * expected_message)
{
  ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  try {
    (void)future.get();
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), expected_message);
  }
}

std::shared_ptr<rclcpp::Node> makeNode(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return std::make_shared<rclcpp::Node>(prefix + "_" + std::to_string(counter.fetch_add(1)));
}

}  // namespace

TEST(RosExecutorQueueTest, ReturnsResultFromSubmittedWork)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  auto future = queue.submit([]() { return 42; });

  const auto result = executor.spin_until_future_complete(future, std::chrono::seconds(1));

  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(future.get(), 42);
}

TEST(RosExecutorQueueTest, RejectsNewWorkAfterShutdown)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_shutdown_test");

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  queue.shutdown();

  auto future = queue.submit([]() {});

  expectRuntimeError(future, "ROS executor queue is shut down.");
}

TEST(RosExecutorQueueTest, ContinuesDrainingQueuedWorkAfterTaskThrows)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_failure_isolation_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());

  auto failed_task = queue.submit([]() -> int { throw std::runtime_error("task failed"); });
  auto trailing_task = queue.submit([]() { return 7; });

  const auto result = executor.spin_until_future_complete(trailing_task, std::chrono::seconds(1));

  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  expectRuntimeError(failed_task, "task failed");
  EXPECT_EQ(trailing_task.get(), 7);
}

TEST(RosExecutorQueueTest, ShutdownWaitsForActiveDrainWorkAndRejectsQueuedOrLateWork)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_active_shutdown_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  std::promise<void> active_task_started_promise;
  auto active_task_started = active_task_started_promise.get_future();
  std::promise<void> release_active_task_promise;
  auto release_active_task = release_active_task_promise.get_future().share();
  std::atomic<bool> queued_task_ran{false};
  std::atomic<bool> late_task_ran{false};

  std::thread executor_thread([&executor]() { executor.spin(); });

  auto active_task = queue.submit([&]() {
    active_task_started_promise.set_value();
    release_active_task.wait();
  });
  auto queued_task = queue.submit([&queued_task_ran]() { queued_task_ran.store(true); });

  ASSERT_EQ(active_task_started.wait_for(std::chrono::seconds(1)), std::future_status::ready);

  std::promise<void> shutdown_finished_promise;
  auto shutdown_finished = shutdown_finished_promise.get_future();
  std::thread shutdown_thread([&]() {
    queue.shutdown();
    shutdown_finished_promise.set_value();
  });

  EXPECT_EQ(shutdown_finished.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  expectRuntimeError(queued_task, "ROS executor queue is shut down.");
  EXPECT_FALSE(queued_task_ran.load());

  auto late_task = queue.submit([&late_task_ran]() { late_task_ran.store(true); });

  expectRuntimeError(late_task, "ROS executor queue is shut down.");
  EXPECT_FALSE(late_task_ran.load());

  release_active_task_promise.set_value();

  ASSERT_EQ(active_task.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_NO_THROW(active_task.get());
  ASSERT_EQ(shutdown_finished.wait_for(std::chrono::seconds(1)), std::future_status::ready);

  shutdown_thread.join();
  executor.cancel();
  executor_thread.join();
}

TEST(RosExecutorQueueTest, ShutdownFromActiveDrainWorkDoesNotDeadlock)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_reentrant_shutdown_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  std::atomic<bool> queued_task_ran{false};

  std::thread executor_thread([&executor]() { executor.spin(); });

  auto active_task = queue.submit([&queue]() { queue.shutdown(); });
  auto queued_task = queue.submit([&queued_task_ran]() { queued_task_ran.store(true); });

  ASSERT_EQ(active_task.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NO_THROW(active_task.get());

  expectRuntimeError(queued_task, "ROS executor queue is shut down.");
  EXPECT_FALSE(queued_task_ran.load());

  executor.cancel();
  executor_thread.join();
}

TEST(RosExecutorQueueTest, ExecutesTasksInSubmissionOrder)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_fifo_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::vector<int> execution_order;
  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());

  queue.submit([&]() { execution_order.push_back(1); });
  queue.submit([&]() { execution_order.push_back(2); });
  auto future = queue.submit([&]() { execution_order.push_back(3); });

  const auto result = executor.spin_until_future_complete(future, std::chrono::seconds(1));

  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NO_THROW(future.get());
  EXPECT_EQ(execution_order, (std::vector<int>{1, 2, 3}));
}

TEST(RosExecutorQueueTest, ExecutesWorkSubmittedFromActiveDrainWork)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_reentrant_submit_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::vector<int> execution_order;
  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  std::promise<void> nested_task_ran_promise;
  auto nested_task_ran = nested_task_ran_promise.get_future();

  auto outer_task = queue.submit([&]() {
    execution_order.push_back(1);
    queue.submit([&]() {
      execution_order.push_back(2);
      nested_task_ran_promise.set_value();
    });
  });

  const auto result = executor.spin_until_future_complete(nested_task_ran, std::chrono::seconds(1));

  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NO_THROW(outer_task.get());
  EXPECT_EQ(execution_order, (std::vector<int>{1, 2}));
}

TEST(RosExecutorQueueTest, WakesExecutorForWorkSubmittedWhileSpinning)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_queue_async_submit_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());

  std::promise<void> spin_started_promise;
  auto spin_started = spin_started_promise.get_future();
  std::promise<std::thread::id> executed_thread_promise;
  auto executed_thread_future = executed_thread_promise.get_future().share();

  std::promise<rclcpp::FutureReturnCode> spin_result_promise;
  auto spin_result_future = spin_result_promise.get_future();

  std::thread executor_thread([&]() {
    spin_started_promise.set_value();
    spin_result_promise.set_value(executor.spin_until_future_complete(executed_thread_future, std::chrono::seconds(1)));
  });

  const auto executor_thread_id = executor_thread.get_id();
  ASSERT_EQ(spin_started.wait_for(std::chrono::seconds(1)), std::future_status::ready);

  auto task_future =
    queue.submit([&executed_thread_promise]() { executed_thread_promise.set_value(std::this_thread::get_id()); });

  const auto spin_status = spin_result_future.wait_for(std::chrono::seconds(2));

  if (spin_status != std::future_status::ready) {
    executor.cancel();
  }
  executor_thread.join();

  ASSERT_EQ(spin_status, std::future_status::ready);
  EXPECT_EQ(spin_result_future.get(), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NO_THROW(task_future.get());
  EXPECT_EQ(executed_thread_future.get(), executor_thread_id);

  queue.shutdown();
}

}  // namespace livekit_ros2_bridge
