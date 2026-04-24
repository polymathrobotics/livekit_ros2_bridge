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

/// Parsed form of a resource-list request. Unknown request fields are ignored.
struct ResourceListRequest
{
  /// Optional trimmed query string. Missing, null, or blank values are treated as absent.
  std::optional<std::string> query;
  /// Optional positive integer result cap. Missing or null means no explicit limit.
  std::optional<std::size_t> limit;
};

/// ROS graph resources keyed by resource name with the interface types reported by rclcpp.
using ResourceNamesAndTypes =
  decltype(std::declval<const rclcpp::node_interfaces::NodeGraphInterface &>().get_topic_names_and_types());

}  // namespace livekit_ros2_bridge
