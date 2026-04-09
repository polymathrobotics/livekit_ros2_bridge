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
#include "rclcpp/node.hpp"
#include "room_session.hpp"

namespace livekit_ros2_bridge
{

class RosExecutorQueue;
class RosServiceCaller;

// Registers the LiveKit RPC surface exposed by this bridge and maps each RPC to
// the ROS-side helper that performs authorization, executor handoff, and
// protocol-level error reporting.
class RpcRouter
{
public:
  RpcRouter(
    rclcpp::Node & node,
    const AccessPolicy & access_policy,
    RosExecutorQueue & ros_executor_queue,
    RosServiceCaller & ros_service_caller);

  // Attempts to register every supported RPC method on the session. Returns
  // false if any individual registration fails; successful handlers remain
  // installed until unregisterRpcMethods() is called.
  bool registerRpcMethods(RoomSession & session);
  void unregisterRpcMethods(RoomSession & session);

private:
  std::optional<std::string> handleServiceCall(const RpcInvocation & invocation);
  std::optional<std::string> handleInterfacesGet(const RpcInvocation & invocation);
  std::optional<std::string> handleServiceList(const RpcInvocation & invocation);
  std::optional<std::string> handleTopicList(const RpcInvocation & invocation);

  rclcpp::Node & node_;
  AccessPolicy access_policy_;
  RosExecutorQueue & ros_executor_queue_;
  RosServiceCaller & ros_service_caller_;
};

}  // namespace livekit_ros2_bridge
