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

#include <string>

#include "rclcpp/serialized_message.hpp"

namespace livekit_ros2_bridge
{

struct RosPublishRequest
{
  /// Relative topics are resolved in RosTopicPublisher's node context before access checks.
  std::string ros_topic;

  std::string interface_type;

  /// Interface-type validation happens at publish time.
  rclcpp::SerializedMessage message;
};

}  // namespace livekit_ros2_bridge
