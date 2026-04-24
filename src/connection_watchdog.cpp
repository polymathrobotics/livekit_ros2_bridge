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
#include <stdexcept>
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

// Long-outage reconnect probes showed that the C++ SDK can surface Reconnecting and then leave the
// bridge alive but stale after the Rust reconnect loop logs terminal failure. In that state the
// bridge did not reliably receive onDisconnected, RoomEos, Reconnected, or a fresh Connected event.
// The watchdog therefore treats SDK lifecycle events as health hints, not as a complete recovery
// contract: only an explicit Connected state clears the budget, and timeout exits the process so the
// supervisor rebuilds a fresh room connection through the bridge's startup-token path.

}  // namespace

ConnectionWatchdog::ConnectionWatchdog(
  RuntimeConfig::HealthConfig config, ConnectionWatchdogNodeInterfaces interfaces, CloseCallback close)
: config_(config)
, logger_(interfaces.get_node_logging_interface()->get_logger())
, close_(std::move(close))
{
  if (!close_) {
    throw std::invalid_argument("ConnectionWatchdog requires a close callback.");
  }

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

ConnectionWatchdog::~ConnectionWatchdog()
{
  timer_.reset();
}

void ConnectionWatchdog::observeConnectionState(livekit::ConnectionState state)
{
  const std::string_view state_name = connectionStateName(state);
  if (state == livekit::ConnectionState::Connected) {
    markHealthy(state_name);
    LogEvent(logger_, "runtime_ready").field("connection_state", state_name).info();
    return;
  }

  markUnhealthy(state_name);

  LogEvent log = LogEvent(logger_, "runtime_disconnect_observed").field("connection_state", state_name);
  if (!config_.watchdog_enabled) {
    log.info();
    return;
  }

  log.field("recovery_timeout_seconds", config_.watchdog_recovery_timeout.count() / 1000.0);
  log.warn();
}

void ConnectionWatchdog::markHealthy(std::string_view reason)
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

  LogEvent(logger_, "runtime_watchdog_healthy")
    .fieldOr("reason", reason, "unknown")
    .field("unhealthy_duration_seconds", *duration)
    .info();
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
      // Do not let repeated SDK reconnecting/disconnected noise extend the outage deadline. The
      // terminal event may never arrive, so the bounded recovery window starts at the first
      // unhealthy observation and ends only when markHealthy() clears it.
      return false;
    }
    unhealthy_ = UnhealthyState{now, now + config_.watchdog_recovery_timeout};
    return true;
  }();

  if (!started) {
    return;
  }

  LogEvent event = LogEvent(logger_, "runtime_watchdog_unhealthy")
                     .fieldOr("reason", reason, "unknown")
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
