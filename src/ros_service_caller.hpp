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
#include <future>
#include <memory>
#include <string>

#include "protocol/services.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"

namespace livekit_ros2_bridge
{

// Owns dynamic ROS service clients and settles each request asynchronously from
// a waitable so RPC handlers can enqueue work without blocking the executor
// thread that created the request.
class RosServiceCaller final
{
public:
  using Response = ServiceCallResponse;

  RosServiceCaller(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables);

  ~RosServiceCaller();

  RosServiceCaller(const RosServiceCaller &) = delete;
  RosServiceCaller & operator=(const RosServiceCaller &) = delete;

  // Starts a service call for the requester. That identity owns the
  // inflight quota slot and is the scope used by cancelForRequester().
  // If request.interface_type is empty, exactly one type must be discoverable
  // for request.service from the current ROS graph. The returned future is
  // ready immediately only for validation, quota, or shutdown failures;
  // otherwise it resolves later from the service response waitable.
  std::future<Response> call(const std::string & requester, const ServiceCallRequest & request);

  void cancelForRequester(const std::string & requester);
  // Fails all inflight calls and drops cached clients and type support so the
  // next call rebuilds from current session and graph state.
  void resetSessionState();

  // Prevents new calls, waits for any active waitable callback to finish, then
  // fails remaining inflight calls. This coordination is reentrant-safe so
  // shutdown can be triggered from code already running inside the waitable.
  void shutdown();

private:
  class Impl;

  std::unique_ptr<Impl> impl_;

  void setWaitableCallbacksForTest(std::function<void()> on_waitable_enter, std::function<void()> on_waitable_exit);
  void setTypeSupportLoadCallbackForTest(std::function<void(const std::string &)> on_type_support_load);
};

}  // namespace livekit_ros2_bridge
