// Copyright (c) 2026-present Polymath Robotics, Inc.
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
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "ros_executor_queue.hpp"
#include "ros_executor_watchdog.hpp"
#include "ros_test_support.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using namespace std::chrono_literals;

constexpr int kScenarioTimedOutWithoutExit = 67;

std::shared_ptr<rclcpp::Node> makeNode(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return std::make_shared<rclcpp::Node>(prefix + "_" + std::to_string(counter.fetch_add(1)));
}

RuntimeConfig::Watchdog makeConfig(std::chrono::milliseconds recovery_timeout = 300ms)
{
  RuntimeConfig::Watchdog config;
  config.enabled = true;
  config.recovery_timeout = recovery_timeout;
  return config;
}

}  // namespace

TEST(RosExecutorWatchdogTest, ExitsWhenExecutorProbeOutlivesRecoveryTimeout)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      test_support::ScopedRclcppInit init;
      auto node = makeNode("ros_executor_watchdog_exit_test");

      RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
      RosExecutorWatchdog watchdog(makeConfig(100ms), queue, rclcpp::get_logger("test_ros_executor_watchdog"));

      watchdog.start();
      std::this_thread::sleep_for(2s);
      std::_Exit(kScenarioTimedOutWithoutExit);
    },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    ".*");
}

TEST(RosExecutorWatchdogTest, ExitsWhenExecutorProbeFails)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      test_support::ScopedRclcppInit init;
      auto node = makeNode("ros_executor_watchdog_failed_probe_test");

      RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
      queue.shutdown();
      RosExecutorWatchdog watchdog(makeConfig(1s), queue, rclcpp::get_logger("test_ros_executor_watchdog"));

      watchdog.start();
      std::this_thread::sleep_for(2s);
      std::_Exit(kScenarioTimedOutWithoutExit);
    },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    ".*");
}

TEST(RosExecutorWatchdogTest, StopsWhileProbeIsPending)
{
  test_support::ScopedRclcppInit init;
  auto node = makeNode("ros_executor_watchdog_pending_stop_test");

  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  RosExecutorWatchdog watchdog(makeConfig(), queue, rclcpp::get_logger("test_ros_executor_watchdog"));

  watchdog.start();
  std::this_thread::sleep_for(50ms);

  auto stopped = std::async(std::launch::async, [&watchdog]() { watchdog.stop(); });

  ASSERT_EQ(stopped.wait_for(1s), std::future_status::ready);
  queue.shutdown();
}

}  // namespace livekit_ros2_bridge
