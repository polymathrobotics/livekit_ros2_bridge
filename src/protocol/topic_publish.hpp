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

/// Runtime form of a ROS publish request carried on a LiveKit data-packet topic.
struct TopicPublishRequest
{
  /// Trimmed ROS topic name from the request. The ROS publisher resolves and validates it
  /// against its node context before policy checks or publishing.
  std::string ros_topic;
  /// Trimmed interface type. Package/msg spelling stays exact for later ROS-graph validation.
  std::string interface_type;
  /// Serialized ROS message payload ready for rclcpp::GenericPublisher.
  rclcpp::SerializedMessage message;
};

}  // namespace livekit_ros2_bridge
