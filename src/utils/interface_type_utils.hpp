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

#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"

namespace livekit_ros2_bridge
{

using RosGraphNamesAndTypes =
  decltype(std::declval<const rclcpp::node_interfaces::NodeGraphInterface &>().get_topic_names_and_types());

// Reject ambiguous ROS graph state at the bridge boundary instead of guessing.
inline std::string requireSingleInterfaceType(
  const RosGraphNamesAndTypes & names_and_types, const std::string & name, const char * resource_kind)
{
  auto it = names_and_types.find(name);
  if (it == names_and_types.end()) {
    throw std::invalid_argument(std::string("No ROS types found for ") + resource_kind + " '" + name + "'.");
  }

  if (it->second.size() != 1) {
    throw std::invalid_argument(std::string("Multiple ROS types found for ") + resource_kind + " '" + name + "'.");
  }
  return it->second.front();
}

}  // namespace livekit_ros2_bridge
