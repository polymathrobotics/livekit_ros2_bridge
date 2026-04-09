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
#include <vector>

namespace livekit_ros2_bridge
{

/// One ROS interface schema entry returned to a remote caller.
struct InterfaceDefinition
{
  /// Fully-qualified ROS interface type such as `sensor_msgs/msg/BatteryState`.
  std::string interface_type;
  /// Stable schema encoding label for `definition`.
  std::string schema_encoding;
  /// Raw `.msg` or `.srv` file contents as read from the ROS package share directory.
  std::string definition;
};

/// Look up a fully-qualified ROS interface type and read its `.msg` or `.srv` definition plus any
/// transitive message dependencies. The requested schema is always returned first, followed by
/// unique dependencies in first-discovery order during recursive traversal.
std::vector<InterfaceDefinition> lookupInterfaceDefinitions(const std::string & interface_type);

}  // namespace livekit_ros2_bridge
