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
#include "room_connection.hpp"

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

  // Registers every supported RPC. Returns false if any registration fails, but
  // successful handlers remain installed until unregisterRpcs() is called.
  // Registered callbacks borrow this router and its dependencies, so
  // unregisterRpcs() must run before any of them are destroyed.
  bool registerRpcs(RoomConnection & connection);
  // Best-effort teardown counterpart to registerRpcs(); call before
  // destroying this router or any borrowed dependency captured by handlers.
  void unregisterRpcs(RoomConnection & connection);

private:
  rclcpp::Node & node_;
  // Copy the policy so registered callbacks do not depend on the caller
  // retaining the original config object for the life of the room connection.
  AccessPolicy access_policy_;
  // Borrowed ROS helpers captured by registered callbacks; unregisterRpcs()
  // must run before either helper or node_ is destroyed.
  RosExecutorQueue & ros_executor_queue_;
  RosServiceCaller & ros_service_caller_;

  std::optional<std::string> callService(const RpcInvocation & invocation);
  std::optional<std::string> getInterfaces(const RpcInvocation & invocation);
  std::optional<std::string> listServices(const RpcInvocation & invocation);
  std::optional<std::string> listTopics(const RpcInvocation & invocation);
};

}  // namespace livekit_ros2_bridge
