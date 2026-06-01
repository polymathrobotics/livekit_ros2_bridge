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

#include "ros_executor_watchdog.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <thread>
#include <utility>

#include "ros_executor_queue.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using namespace std::chrono_literals;

constexpr auto kProbeInterval = 5s;
constexpr auto kStallAfter = 2s;
constexpr auto kPollInterval = 100ms;
constexpr auto kExitDelay = 100ms;

template <typename Duration>
long long milliseconds(Duration duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

template <typename Duration>
double seconds(Duration duration)
{
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

RosExecutorWatchdog::RosExecutorWatchdog(
  RuntimeConfig::Watchdog config, RosExecutorQueue & queue, rclcpp::Logger logger)
: config_(config)
, queue_(queue)
, logger_(std::move(logger))
{}

RosExecutorWatchdog::~RosExecutorWatchdog()
{
  stop();
}

void RosExecutorWatchdog::start()
{
  if (!config_.enabled) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_.joinable() || stop_requested_) {
    return;
  }

  thread_ = std::thread([this]() { run(); });
}

void RosExecutorWatchdog::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  wake_.notify_all();

  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RosExecutorWatchdog::stopRequested()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_requested_;
}

bool RosExecutorWatchdog::waitForStop(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return wake_.wait_for(lock, timeout, [this]() { return stop_requested_; });
}

void RosExecutorWatchdog::run()
{
  while (!stopRequested()) {
    probe();

    if (waitForStop(kProbeInterval)) {
      break;
    }
  }
}

void RosExecutorWatchdog::probe()
{
  const auto started_at = SteadyClock::now();
  auto completion = queue_.submit([]() {});

  bool unhealthy_logged = false;

  while (completion.wait_for(kPollInterval) != std::future_status::ready) {
    if (stopRequested()) {
      return;
    }

    const auto now = SteadyClock::now();
    const auto pending = now - started_at;
    if (pending >= config_.recovery_timeout) {
      LogEvent(logger_, "ros_executor_watchdog_shutdown")
        .field("shutdown_reason", "recovery_timeout")
        .field("pending_ms", milliseconds(pending))
        .field("recovery_timeout_seconds", seconds(config_.recovery_timeout))
        .error();

      std::this_thread::sleep_for(kExitDelay);
      std::_Exit(EXIT_FAILURE);
    }

    if (pending < kStallAfter) {
      continue;
    }

    if (!unhealthy_logged) {
      unhealthy_logged = true;
      LogEvent(logger_, "ros_executor_watchdog_unhealthy")
        .field("reason", "probe_timeout")
        .field("pending_ms", milliseconds(pending))
        .field("recovery_timeout_seconds", seconds(config_.recovery_timeout))
        .warn();
    }
  }

  try {
    completion.get();
  } catch (...) {
    if (stopRequested()) {
      return;
    }

    const auto pending = SteadyClock::now() - started_at;
    LogEvent(logger_, "ros_executor_watchdog_shutdown")
      .field("shutdown_reason", "probe_failed")
      .field("pending_ms", milliseconds(pending))
      .field("recovery_timeout_seconds", seconds(config_.recovery_timeout))
      .fieldException("error", std::current_exception())
      .error();

    std::this_thread::sleep_for(kExitDelay);
    std::_Exit(EXIT_FAILURE);
  }

  if (unhealthy_logged) {
    const auto unhealthy_duration = SteadyClock::now() - started_at;
    LogEvent(logger_, "ros_executor_watchdog_recovered")
      .field("unhealthy_duration_seconds", seconds(unhealthy_duration))
      .info();
  }
}

}  // namespace livekit_ros2_bridge
