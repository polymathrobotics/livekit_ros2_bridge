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
#include <utility>

#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kExitDelay = std::chrono::milliseconds(100);
constexpr std::string_view kStartupPendingReason = "startup_connect_pending";

}  // namespace

ConnectionWatchdog::ConnectionWatchdog(RuntimeConfig::Watchdog config, rclcpp::Logger logger)
: config_(config)
, logger_(std::move(logger))
{
  if (!config_.enabled) {
    return;
  }

  thread_ = std::thread([this]() { run(); });
  startOutage(kStartupPendingReason);
}

ConnectionWatchdog::~ConnectionWatchdog()
{
  stop();
}

void ConnectionWatchdog::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    outage_.reset();
  }
  wake_.notify_all();

  if (thread_.joinable()) {
    thread_.join();
  }
}

void ConnectionWatchdog::onStateChanged(livekit::ConnectionState state)
{
  if (state == livekit::ConnectionState::Connected) {
    clearOutage();
    return;
  }

  startOutage(state);
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

  wake_.notify_all();
  LogEvent(logger_, "connection_watchdog_recovered").field("unhealthy_duration_seconds", *duration).info();
}

bool ConnectionWatchdog::startOutageTimer()
{
  if (!config_.enabled) {
    return false;
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

  if (started) {
    wake_.notify_all();
  }
  return started;
}

void ConnectionWatchdog::startOutage(std::string_view reason)
{
  if (!startOutageTimer()) {
    return;
  }

  LogEvent(logger_, "connection_watchdog_unhealthy")
    .field("reason", reason)
    .field("recovery_timeout_seconds", config_.recovery_timeout.count() / 1000.0)
    .info();
}

void ConnectionWatchdog::startOutage(livekit::ConnectionState state)
{
  if (!startOutageTimer()) {
    return;
  }

  LogEvent(logger_, "connection_watchdog_unhealthy")
    .fieldEnum("connection_state", state)
    .field("recovery_timeout_seconds", config_.recovery_timeout.count() / 1000.0)
    .warn();
}

void ConnectionWatchdog::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_requested_) {
    if (!outage_.has_value()) {
      wake_.wait(lock, [this]() { return stop_requested_ || outage_.has_value(); });
      continue;
    }

    const auto deadline = outage_->deadline;
    (void)wake_.wait_until(lock, deadline);

    if (stop_requested_ || !outage_.has_value() || SteadyClock::now() < outage_->deadline) {
      continue;
    }

    outage_.reset();
    lock.unlock();

    LogEvent(logger_, "connection_watchdog_shutdown")
      .field("shutdown_reason", "recovery_timeout")
      .field("recovery_timeout_seconds", config_.recovery_timeout.count() / 1000.0)
      .error();

    std::this_thread::sleep_for(kExitDelay);
    std::_Exit(EXIT_FAILURE);
  }
}

}  // namespace livekit_ros2_bridge
