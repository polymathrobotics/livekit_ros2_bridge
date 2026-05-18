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

#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string_view>
#include <thread>

#include "livekit/room_event_types.h"
#include "rclcpp/logger.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{

class ConnectionWatchdog final
{
public:
  using SteadyClock = std::chrono::steady_clock;

  ConnectionWatchdog(RuntimeConfig::Watchdog config, rclcpp::Logger logger);
  ~ConnectionWatchdog();

  ConnectionWatchdog(const ConnectionWatchdog &) = delete;
  ConnectionWatchdog & operator=(const ConnectionWatchdog &) = delete;
  ConnectionWatchdog(ConnectionWatchdog &&) = delete;
  ConnectionWatchdog & operator=(ConnectionWatchdog &&) = delete;

  void onStateChanged(livekit::ConnectionState state);
  void stop();

private:
  void clearOutage();
  bool startOutageTimer();
  void startOutage(std::string_view reason);
  void startOutage(livekit::ConnectionState state);
  void run();

  RuntimeConfig::Watchdog config_;
  rclcpp::Logger logger_;

  struct Outage
  {
    SteadyClock::time_point since;
    SteadyClock::time_point deadline;
  };

  // LiveKit state callbacks and the watchdog thread can run concurrently.
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_requested_ = false;
  std::optional<Outage> outage_;
  std::thread thread_;
};

}  // namespace livekit_ros2_bridge
