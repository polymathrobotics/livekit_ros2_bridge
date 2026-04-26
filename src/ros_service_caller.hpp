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

#include <future>
#include <memory>
#include <string>

#include "protocol/services.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"

namespace livekit_ros2_bridge
{

// Owns runtime-discovered ROS service clients and settles requests from a waitable.
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

  // `requester` scopes inflight quota and cancelForRequester().
  // Empty request.interface_type resolves from the ROS graph and must match one type.
  std::future<Response> call(const std::string & requester, const ServiceCallRequest & request);

  void cancelForRequester(const std::string & requester);

  // Stops new waitable work, waits for the active callback, then fails inflight calls.
  void shutdown();

private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace livekit_ros2_bridge
