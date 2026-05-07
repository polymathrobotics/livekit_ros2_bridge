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
#include <exception>
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
constexpr auto kExitDelay = std::chrono::milliseconds(100);
constexpr std::string_view kStartupPendingReason = "startup_connect_pending";

std::string_view stateName(livekit::ConnectionState state)
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

ConnectionWatchdog::ConnectionWatchdog(RuntimeConfig::Watchdog config, NodeInterfaces interfaces, CloseCallback close)
: config_(config)
, logger_(interfaces.get_node_logging_interface()->get_logger())
, close_(std::move(close))
{
  if (!config_.enabled) {
    return;
  }

  timer_ = rclcpp::create_wall_timer(
    kCheckInterval,
    [this]() { check(); },
    nullptr,
    interfaces.get_node_base_interface().get(),
    interfaces.get_node_timers_interface().get());
  startOutage(kStartupPendingReason);
}

void ConnectionWatchdog::onStateChanged(livekit::ConnectionState state)
{
  if (state == livekit::ConnectionState::Connected) {
    clearOutage();
    return;
  }

  startOutage(stateName(state));
}

void ConnectionWatchdog::clearOutage()
{
  if (!config_.enabled) {
    return;
  }

  const auto now = SteadyClock::now();
  const std::optional<double> duration = [&]() -> std::optional<double> {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!outage_.has_value()) {
      return std::nullopt;
    }
    const double seconds = std::chrono::duration<double>(now - outage_->since).count();
    outage_.reset();
    return seconds;
  }();

  if (!duration.has_value()) {
    return;
  }

  LogEvent(logger_, "connection_watchdog_recovered").field("unhealthy_duration_seconds", *duration).info();
}

void ConnectionWatchdog::startOutage(std::string_view reason)
{
  if (!config_.enabled) {
    return;
  }

  const auto now = SteadyClock::now();
  const bool started = [&]() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (outage_.has_value()) {
      // Do not extend outage deadlines; reconnect failure may never emit a terminal event.
      return false;
    }
    outage_ = Outage{now, now + config_.recovery_timeout};
    return true;
  }();

  if (!started) {
    return;
  }

  LogEvent event = LogEvent(logger_, "connection_watchdog_unhealthy")
                     .field("reason", reason)
                     .field("recovery_timeout_seconds", config_.recovery_timeout.count() / 1000.0);
  if (reason == kStartupPendingReason) {
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
    if (!outage_.has_value()) {
      return;
    }
    if (now < outage_->deadline) {
      return;
    }
    outage_.reset();
  }

  LogEvent(logger_, "connection_watchdog_shutdown")
    .field("shutdown_reason", "recovery_timeout")
    .field("recovery_timeout_seconds", config_.recovery_timeout.count() / 1000.0)
    .error();

  try {
    auto close = close_;
    auto logger = logger_;
    std::thread([close = std::move(close), logger]() mutable {
      try {
        (void)close();
      } catch (...) {
        LogEvent(logger, "connection_watchdog_close_failed").fieldException("error", std::current_exception()).error();
      }
    }).detach();
  } catch (...) {
    LogEvent(logger_, "connection_watchdog_close_failed").fieldException("error", std::current_exception()).error();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  std::this_thread::sleep_for(kExitDelay);
  std::_Exit(EXIT_FAILURE);
}

}  // namespace livekit_ros2_bridge
