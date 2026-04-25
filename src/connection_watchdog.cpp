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

#include "connection_watchdog.hpp"

#include <cstdlib>
#include <thread>
#include <utility>

#include "rclcpp/create_timer.hpp"
#include "rclcpp/utilities.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kCheckInterval = std::chrono::milliseconds(250);
constexpr auto kShutdownExitDelay = std::chrono::milliseconds(100);
constexpr std::string_view kStartupConnectPendingReason = "startup_connect_pending";

std::string_view connectionStateName(livekit::ConnectionState state)
{
  switch (state) {
    case livekit::ConnectionState::Disconnected:
      return "disconnected";
    case livekit::ConnectionState::Connected:
      return "connected";
    case livekit::ConnectionState::Reconnecting:
      return "reconnecting";
  }

  return "unknown";
}

}  // namespace

ConnectionWatchdog::ConnectionWatchdog(
  RuntimeConfig::HealthConfig config, ConnectionWatchdogNodeInterfaces interfaces, CloseCallback close)
: config_(config)
, logger_(interfaces.get_node_logging_interface()->get_logger())
, close_(std::move(close))
{
  if (!config_.watchdog_enabled) {
    return;
  }

  timer_ = rclcpp::create_wall_timer(
    kCheckInterval,
    [this]() { check(); },
    nullptr,
    interfaces.get_node_base_interface().get(),
    interfaces.get_node_timers_interface().get());
  markUnhealthy(kStartupConnectPendingReason);
}

void ConnectionWatchdog::observeConnectionState(livekit::ConnectionState state)
{
  if (state == livekit::ConnectionState::Connected) {
    markHealthy();
    return;
  }

  const std::string_view state_name = connectionStateName(state);
  markUnhealthy(state_name);
}

void ConnectionWatchdog::markHealthy()
{
  if (!config_.watchdog_enabled) {
    return;
  }

  const auto now = SteadyClock::now();
  const std::optional<double> duration = [&]() -> std::optional<double> {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!unhealthy_.has_value()) {
      return std::nullopt;
    }
    const double seconds = std::chrono::duration<double>(now - unhealthy_->since).count();
    unhealthy_.reset();
    return seconds;
  }();

  if (!duration.has_value()) {
    return;
  }

  LogEvent(logger_, "runtime_watchdog_healthy").field("unhealthy_duration_seconds", *duration).info();
}

void ConnectionWatchdog::markUnhealthy(std::string_view reason)
{
  if (!config_.watchdog_enabled) {
    return;
  }

  const auto now = SteadyClock::now();
  const bool started = [&]() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (unhealthy_.has_value()) {
      // Do not extend outage deadlines; reconnect failure may never emit a terminal event.
      return false;
    }
    unhealthy_ = UnhealthyState{now, now + config_.watchdog_recovery_timeout};
    return true;
  }();

  if (!started) {
    return;
  }

  LogEvent event = LogEvent(logger_, "runtime_watchdog_unhealthy")
                     .field("reason", reason)
                     .field("recovery_timeout_seconds", config_.watchdog_recovery_timeout.count() / 1000.0);
  if (reason == kStartupConnectPendingReason) {
    event.info();
    return;
  }
  event.warn();
}

void ConnectionWatchdog::check()
{
  const auto now = SteadyClock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!unhealthy_.has_value()) {
      return;
    }
    if (now < unhealthy_->deadline) {
      return;
    }
  }

  if (!close_()) {
    return;
  }

  LogEvent(logger_, "runtime_watchdog_triggered")
    .field("disconnect_reason", "recovery_timeout")
    .field("recovery_timeout_seconds", config_.watchdog_recovery_timeout.count() / 1000.0)
    .error();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  std::this_thread::sleep_for(kShutdownExitDelay);
  std::_Exit(EXIT_FAILURE);
}

}  // namespace livekit_ros2_bridge
