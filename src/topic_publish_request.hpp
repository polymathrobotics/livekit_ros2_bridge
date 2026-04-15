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

/// Parsed form of a `ros.topics.publish` request carried in a LiveKit data packet.
struct TopicPublishRequest
{
  /// Normalized absolute topic name.
  std::string topic;
  /// Trimmed interface type. Package/msg spelling stays exact for later ROS-graph validation.
  std::string interface_type;
  std::vector<std::uint8_t> cdr;
};

/// Parse a JSON topic-publish request. Normalizes `topic`, trims `interface_type` without
/// rewriting its package/msg spelling, and requires a non-empty CDR `message` payload. Throws
/// `std::invalid_argument` on malformed JSON or protocol-contract violations.
TopicPublishRequest parseTopicPublishRequest(const std::vector<std::uint8_t> & payload);

}  // namespace livekit_ros2_bridge
