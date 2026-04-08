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

struct InterfaceDefinition
{
  std::string interface_type;
  std::string schema_encoding;
  std::string definition;
};

struct InterfaceDefinitions
{
  InterfaceDefinition requested;
  std::vector<InterfaceDefinition> dependencies;
};

/// Look up the requested ROS interface definition and its transitive dependency definitions for a
/// fully-qualified ROS interface type.
///
/// @param interface_type  Fully-qualified ROS interface type, e.g. "sensor_msgs/msg/BatteryState"
/// @throws std::invalid_argument if the interface type name is malformed
/// @throws std::runtime_error if the interface definition cannot be found or read
InterfaceDefinitions lookupInterfaceDefinitions(const std::string & interface_type);

}  // namespace livekit_ros2_bridge
