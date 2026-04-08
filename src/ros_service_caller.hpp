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
class RosServiceCallerTestPeer;

class RosServiceCaller final
{
public:
  static constexpr int kDefaultTimeoutMs = 2000;
  static constexpr int kMaxInflightPerRequester = 4;

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

  std::future<ServiceCallResponse> call(const std::string & requester_identity, const ServiceCallRequest & request);

  void cancelCallsForRequester(const std::string & requester_identity);
  void resetSessionState();

  void shutdown();

private:
  friend class RosServiceCallerTestPeer;

  void setPollCallbackHooksForTest(std::function<void()> on_enter, std::function<void()> on_exit);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livekit_ros2_bridge
