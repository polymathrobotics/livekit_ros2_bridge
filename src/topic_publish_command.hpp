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
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

/// Parsed form of a `ros.topics.publish` control message.
struct TopicPublishCommand
{
  /// Normalized absolute topic name from the required `topic` field.
  std::string topic;
  /// Trimmed ROS interface type from the required `interface_type` field.
  std::string interface_type;
  /// Raw CDR bytes from the required non-empty `message` payload object.
  std::vector<std::uint8_t> cdr_payload;
};

/// Parse a JSON command object with required `topic`, `interface_type`, and `message` fields.
/// `topic` is normalized as a ROS resource name and `message` must use the stable CDR payload schema.
TopicPublishCommand parseTopicPublishCommand(const std::vector<std::uint8_t> & command_payload);

}  // namespace livekit_ros2_bridge
