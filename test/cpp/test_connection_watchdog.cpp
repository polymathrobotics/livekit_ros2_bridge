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

#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "livekit/room_event_types.h"
#include "rclcpp/logger.hpp"
#include "runtime_config.hpp"

#define private public
#include "connection_watchdog.hpp"
#undef private

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kRecoveryTimeout = std::chrono::milliseconds(300);
constexpr auto kWatchdogObservationWindow = std::chrono::seconds(2);
constexpr auto kWatchdogThreadStartGrace = std::chrono::milliseconds(60);
constexpr auto kRestartedOutageExtension = std::chrono::milliseconds(700);
constexpr auto kStopAfterOriginalDeadline = std::chrono::milliseconds(250);
constexpr int kMissingOutageExitCode = 66;
constexpr int kWatchdogScenarioTimedOutWithoutExit = 67;

RuntimeConfig::Watchdog makeWatchdogConfig()
{
  RuntimeConfig::Watchdog config;
  config.enabled = true;
  config.recovery_timeout = kRecoveryTimeout;
  return config;
}

}  // namespace

TEST(ConnectionWatchdogTest, ExitsWhenReconnectOutageOutlivesBlockedCaller)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      ConnectionWatchdog watchdog(makeWatchdogConfig(), rclcpp::get_logger("test_connection_watchdog"));
      watchdog.onStateChanged(livekit::ConnectionState::Connected);
      watchdog.onStateChanged(livekit::ConnectionState::Reconnecting);

      std::this_thread::sleep_for(kWatchdogObservationWindow);
      std::_Exit(kWatchdogScenarioTimedOutWithoutExit);
    },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    ".*");
}

TEST(ConnectionWatchdogTest, RestartedOutageGetsFullRecoveryTimeout)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      ConnectionWatchdog watchdog(makeWatchdogConfig(), rclcpp::get_logger("test_connection_watchdog"));
      std::this_thread::sleep_for(kWatchdogThreadStartGrace);

      ConnectionWatchdog::SteadyClock::time_point original_deadline;
      {
        std::lock_guard<std::mutex> lock(watchdog.mutex_);
        if (!watchdog.outage_.has_value()) {
          std::_Exit(kMissingOutageExitCode);
        }
        original_deadline = watchdog.outage_->deadline;
      }

      {
        std::lock_guard<std::mutex> lock(watchdog.mutex_);
        const auto restarted_since = ConnectionWatchdog::SteadyClock::now();
        watchdog.outage_.reset();
        watchdog.wake_.notify_all();
        ConnectionWatchdog::Outage restarted_outage;
        restarted_outage.since = restarted_since;
        restarted_outage.deadline = original_deadline + kRestartedOutageExtension;
        watchdog.outage_ = restarted_outage;
        watchdog.wake_.notify_all();
      }

      std::this_thread::sleep_until(original_deadline + kStopAfterOriginalDeadline);
      watchdog.stop();
      std::_Exit(EXIT_SUCCESS);
    },
    ::testing::ExitedWithCode(EXIT_SUCCESS),
    ".*");
}

}  // namespace livekit_ros2_bridge
