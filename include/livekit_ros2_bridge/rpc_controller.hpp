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

#include "livekit_ros2_bridge/access_policy.hpp"
#include "livekit_ros2_bridge/livekit_ros2_bridge_parameters.hpp"
#include "livekit_ros2_bridge/livekit_session.hpp"
#include "rclcpp/logger.hpp"

namespace livekit_ros2_bridge
{

class RpcController
{
public:
  RpcController(rclcpp::Logger logger, const Params & params);

  void registerMethods(LiveKitSession & session);
  void unregisterMethods(LiveKitSession & session);

private:
  std::optional<std::string> handleTopicSubscribe(const RpcInvocation & invocation) const;
  std::optional<std::string> handleServiceCall(const RpcInvocation & invocation) const;

  rclcpp::Logger logger_;
  StaticAccessPolicy access_policy_;
  bool methods_registered_ = false;
};

}  // namespace livekit_ros2_bridge
