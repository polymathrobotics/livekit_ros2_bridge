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

#include <functional>
#include <memory>
#include <string>

#include "connection_watchdog.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "room_connection.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "runtime_config.hpp"
#include "subscription_lease_manager.hpp"
#include "utils/callback_gate.hpp"

namespace livekit_ros2_bridge
{

class Runtime final
{
public:
  using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeGraphInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTimersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeWaitablesInterface>;

  Runtime(NodeInterfaces interfaces, std::unique_ptr<RoomConnection> connection, RuntimeConfig config);
  ~Runtime();

  Runtime(const Runtime &) = delete;
  Runtime & operator=(const Runtime &) = delete;
  Runtime(Runtime &&) = delete;
  Runtime & operator=(Runtime &&) = delete;

private:
  void onUserPacketReceived(const livekit::UserDataPacketEvent & event);
  void submitRosWork(std::function<void()> work);
  RoomEventCallbacks makeRoomCallbacks();

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  RuntimeConfig config_;
  CallbackGate callback_gate_;
  std::unique_ptr<RoomConnection> room_connection_;
  RosExecutorQueue ros_executor_queue_;
  RosTopicPublisher ros_topic_publisher_;
  RosServiceCaller ros_service_caller_;
  SubscriptionLeaseManager subscription_lease_manager_;
  RpcRouter rpc_router_;
  ConnectionWatchdog watchdog_;
};

}  // namespace livekit_ros2_bridge
