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

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "connection_watchdog.hpp"
#include "packet_router.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "runtime_config.hpp"
#include "subscription_lease_manager.hpp"

namespace livekit_ros2_bridge
{

struct RuntimeNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Logger logger;

  static RuntimeNodeInterfaces fromNode(rclcpp::Node & node);
};

class RuntimeCallbackGate final
{
public:
  template <typename Fn>
  bool runIfOpen(Fn && fn)
  {
    if (!tryEnter()) {
      return false;
    }

    struct ActiveDispatch
    {
      RuntimeCallbackGate & gate;

      ~ActiveDispatch()
      {
        gate.leave();
      }
    } active_dispatch{*this};

    std::forward<Fn>(fn)();
    return true;
  }

  bool closeAndWait();

private:
  std::mutex mutex_;
  std::condition_variable idle_;
  bool closed_ = false;
  std::size_t active_count_ = 0U;

  bool tryEnter();
  void leave();
};

class Runtime final
{
public:
  Runtime(RuntimeNodeInterfaces interfaces, std::unique_ptr<RoomConnection> connection, RuntimeConfig config);
  ~Runtime();

  Runtime(const Runtime &) = delete;
  Runtime & operator=(const Runtime &) = delete;
  Runtime(Runtime &&) = delete;
  Runtime & operator=(Runtime &&) = delete;

private:
  void onRoomConnected();
  void onRoomIncomingPacket(const IncomingPacket & packet);
  void onRoomRemoteParticipantDisconnected(std::string remote_participant_identity);
  void onRoomReconnectRequested(const std::string & reason);
  void onRoomReconnecting(const std::string & reason);
  void onRoomReconnected();
  void onRoomConnectionReset();
  void submitToExecutor(std::function<void()> work);
  RoomEventCallbacks makeRoomEventCallbacks();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  RuntimeConfig config_;
  RuntimeCallbackGate callback_gate_;
  std::unique_ptr<RoomConnection> room_connection_;
  RosExecutorQueue ros_executor_queue_;
  RosTopicPublisher ros_topic_publisher_;
  RosServiceCaller ros_service_caller_;
  SubscriptionLeaseManager subscription_lease_manager_;
  PacketRouter packet_router_;
  RpcRouter rpc_router_;
  ConnectionWatchdog watchdog_;
};

}  // namespace livekit_ros2_bridge
