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

#include <optional>
#include <string>

#include "access_policy.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "room_connection.hpp"

namespace livekit_ros2_bridge
{

class RosExecutorQueue;
class RosServiceCaller;
class SubscriptionLeaseManager;

class RpcRouter
{
public:
  RpcRouter(
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    const AccessPolicy & policy,
    RosExecutorQueue & queue,
    RosServiceCaller & caller,
    SubscriptionLeaseManager & lease_manager,
    bool audio_output_enabled = false);
  ~RpcRouter();

  RpcRouter(const RpcRouter &) = delete;
  RpcRouter & operator=(const RpcRouter &) = delete;
  RpcRouter(RpcRouter &&) = delete;
  RpcRouter & operator=(RpcRouter &&) = delete;

  // Handlers borrow this router until unregistration/destruction; false means any method failed.
  bool registerRpcs(RoomConnection & connection);

  // Idempotently unregisters methods from the most recent connection.
  void unregisterRpcs() noexcept;

private:
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  // Registered callbacks must not depend on caller-owned config.
  AccessPolicy policy_;
  // Borrowed by registered callbacks; must outlive this router.
  RosExecutorQueue & queue_;
  RosServiceCaller & caller_;
  SubscriptionLeaseManager & lease_manager_;
  // Borrowed only for unregistration; must outlive unregisterRpcs()/destruction.
  RoomConnection * registered_connection_ = nullptr;

  std::optional<std::string> callService(const livekit::RpcInvocationData & invocation);
  std::optional<std::string> showInterfaces(const livekit::RpcInvocationData & invocation);
  std::optional<std::string> listServices(const livekit::RpcInvocationData & invocation);
  std::optional<std::string> listTopics(const livekit::RpcInvocationData & invocation);
  std::optional<std::string> requestEchoOnce(const livekit::RpcInvocationData & invocation);
  std::optional<std::string> capability(const livekit::RpcInvocationData & invocation);

  bool audio_output_enabled_ = false;
};

}  // namespace livekit_ros2_bridge
