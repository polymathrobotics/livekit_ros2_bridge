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

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace livekit_ros2_bridge
{

struct ServiceCallRequest;

// Owns dynamic ROS service clients and settles each request asynchronously from
// a poll timer so RPC handlers can enqueue work without blocking the executor
// thread that created the request.
class RosServiceCaller final
{
public:
  // The payload stays serialized so callers can forward arbitrary service types
  // without templating RosServiceCaller on generated ROS interfaces.
  struct ServiceCallResponse
  {
    std::string service;
    std::string interface_type;
    std::vector<std::uint8_t> response;
  };

  explicit RosServiceCaller(rclcpp::Node & node);

  ~RosServiceCaller();

  RosServiceCaller(const RosServiceCaller &) = delete;
  RosServiceCaller & operator=(const RosServiceCaller &) = delete;

  // Starts a service call for the requester. That identity owns the
  // inflight quota slot and is the scope used by cancelCallsForRequester().
  // If request.interface_type is empty, exactly one type must be discoverable
  // for request.service from the current ROS graph. The returned future is
  // ready immediately only for validation, quota, or shutdown failures;
  // otherwise it resolves later from the poll timer.
  std::future<ServiceCallResponse> call(const std::string & requester, const ServiceCallRequest & request);

  void cancelCallsForRequester(const std::string & requester);
  // Fails all pending calls and drops cached clients and type support so the
  // next call rebuilds from current session and graph state.
  void resetSessionState();

  // Prevents new calls, waits for any active poll callback to finish, then
  // fails remaining pending calls. This coordination is reentrant-safe so
  // shutdown can be triggered from code already running inside poll().
  void shutdown();

private:
  void setPollCallbacksForTest(std::function<void()> on_poll_enter, std::function<void()> on_poll_exit);
  void setTypeSupportLoadCallbackForTest(std::function<void(const std::string &)> on_type_support_load);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livekit_ros2_bridge
