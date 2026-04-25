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
#include <functional>
#include <mutex>
#include <optional>
#include <string_view>

#include "livekit/room_event_types.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/timer.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{

class ConnectionWatchdog final
{
public:
  using CloseCallback = std::function<bool()>;
  using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeTimersInterface>;
  using SteadyClock = std::chrono::steady_clock;

  ConnectionWatchdog(RuntimeConfig::Watchdog config, NodeInterfaces interfaces, CloseCallback close);

  ConnectionWatchdog(const ConnectionWatchdog &) = delete;
  ConnectionWatchdog & operator=(const ConnectionWatchdog &) = delete;
  ConnectionWatchdog(ConnectionWatchdog &&) = delete;
  ConnectionWatchdog & operator=(ConnectionWatchdog &&) = delete;

  void onStateChanged(livekit::ConnectionState state);

private:
  void clearOutage();
  void startOutage(std::string_view reason);

  RuntimeConfig::Watchdog config_;
  rclcpp::Logger logger_;
  CloseCallback close_;

  struct Outage
  {
    SteadyClock::time_point since;
    SteadyClock::time_point deadline;
  };

  // LiveKit state callbacks and the ROS timer can run on different threads.
  std::mutex mutex_;
  std::optional<Outage> outage_;
  rclcpp::TimerBase::SharedPtr timer_;

  void check();
};

}  // namespace livekit_ros2_bridge
