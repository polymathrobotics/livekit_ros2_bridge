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

#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "rclcpp/logger.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{

class RosExecutorQueue;

class RosExecutorWatchdog final
{
public:
  RosExecutorWatchdog(RuntimeConfig::Watchdog config, RosExecutorQueue & queue, rclcpp::Logger logger);
  ~RosExecutorWatchdog();

  RosExecutorWatchdog(const RosExecutorWatchdog &) = delete;
  RosExecutorWatchdog & operator=(const RosExecutorWatchdog &) = delete;
  RosExecutorWatchdog(RosExecutorWatchdog &&) = delete;
  RosExecutorWatchdog & operator=(RosExecutorWatchdog &&) = delete;

  void start();
  void stop();

private:
  using SteadyClock = std::chrono::steady_clock;

  bool stopRequested();
  bool waitForStop(std::chrono::milliseconds timeout);
  void run();
  void probe();

  RuntimeConfig::Watchdog config_;
  RosExecutorQueue & queue_;
  rclcpp::Logger logger_;

  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_requested_ = false;
  std::thread thread_;
};

}  // namespace livekit_ros2_bridge
