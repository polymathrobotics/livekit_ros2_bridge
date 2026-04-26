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

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace livekit_ros2_bridge::ros_interfaces
{

// Reject ambiguous ROS graph state at the bridge boundary instead of guessing.
inline std::string requireSingleType(
  const std::map<std::string, std::vector<std::string>> & graph, const std::string & resource, const char * kind)
{
  const auto found = graph.find(resource);
  if (found == graph.end()) {
    throw std::invalid_argument(std::string("No ROS types found for ") + kind + " '" + resource + "'.");
  }

  if (found->second.size() != 1) {
    throw std::invalid_argument(std::string("Multiple ROS types found for ") + kind + " '" + resource + "'.");
  }
  return found->second.front();
}

}  // namespace livekit_ros2_bridge::ros_interfaces
