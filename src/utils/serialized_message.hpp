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

#ifndef LIVEKIT_ROS2_BRIDGE__SERIALIZED_MESSAGE_HPP_
#define LIVEKIT_ROS2_BRIDGE__SERIALIZED_MESSAGE_HPP_

#include <cstdint>
#include <vector>

#include "rcl/allocator.h"
#include "rclcpp/serialized_message.hpp"
#include "rmw/serialized_message.h"

namespace livekit_ros2_bridge
{

inline rclcpp::SerializedMessage wrapSerializedPayload(const std::vector<std::uint8_t> & payload)
{
  auto source = rmw_get_zero_initialized_serialized_message();
  source.buffer = payload.empty() ? nullptr : const_cast<std::uint8_t *>(payload.data());
  source.buffer_length = payload.size();
  source.buffer_capacity = payload.size();
  source.allocator = rcl_get_default_allocator();
  return rclcpp::SerializedMessage(source);
}

}  // namespace livekit_ros2_bridge

#endif  // LIVEKIT_ROS2_BRIDGE__SERIALIZED_MESSAGE_HPP_
