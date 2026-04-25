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

#include <cstddef>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"

namespace livekit_ros2_bridge
{

struct ResourceListRequest
{
  /// Trimmed substring filter; absent when omitted, null, or blank.
  std::optional<std::string> query;

  /// Positive result cap; nullopt means uncapped.
  std::optional<std::size_t> limit;
};

using ResourceTypesByName =
  decltype(std::declval<const rclcpp::node_interfaces::NodeGraphInterface &>().get_topic_names_and_types());

}  // namespace livekit_ros2_bridge
