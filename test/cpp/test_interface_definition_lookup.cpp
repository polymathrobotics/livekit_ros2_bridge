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

#include <set>
#include <stdexcept>

#include "gtest/gtest.h"
#include "interface_definition_lookup.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::set<std::string> dependencyTypes(const InterfaceDefinitions & definitions)
{
  std::set<std::string> types;
  for (const auto & dependency : definitions.dependencies) {
    types.insert(dependency.interface_type);
  }
  return types;
}

TEST(LookupInterfaceDefinitionTest, LooksUpSimpleMessageWithoutDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_msgs/msg/String");

  EXPECT_EQ(result.requested.interface_type, "std_msgs/msg/String");
  EXPECT_EQ(result.requested.schema_encoding, "ros2msg");
  EXPECT_NE(result.requested.definition.find("string data"), std::string::npos);
  EXPECT_TRUE(result.dependencies.empty());
}

TEST(LookupInterfaceDefinitionTest, LooksUpMessageWithDirectDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_msgs/msg/Header");

  EXPECT_EQ(result.requested.interface_type, "std_msgs/msg/Header");
  EXPECT_NE(result.requested.definition.find("builtin_interfaces/Time stamp"), std::string::npos);

  ASSERT_EQ(result.dependencies.size(), 1u);
  EXPECT_EQ(result.dependencies.front().interface_type, "builtin_interfaces/msg/Time");
  EXPECT_EQ(result.dependencies.front().schema_encoding, "ros2msg");
  EXPECT_NE(result.dependencies.front().definition.find("int32 sec"), std::string::npos);
}

TEST(LookupInterfaceDefinitionTest, LooksUpTransitiveDependenciesWithoutDuplicates)
{
  const auto result = lookupInterfaceDefinitions("sensor_msgs/msg/BatteryState");

  EXPECT_EQ(result.requested.interface_type, "sensor_msgs/msg/BatteryState");
  const std::set<std::string> expected_dependencies = {
    "builtin_interfaces/msg/Time",
    "std_msgs/msg/Header",
  };
  EXPECT_EQ(dependencyTypes(result), expected_dependencies);
  EXPECT_EQ(result.dependencies.size(), expected_dependencies.size());
}

TEST(LookupInterfaceDefinitionTest, LooksUpPrimitiveOnlyServiceWithoutDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_srvs/srv/SetBool");

  EXPECT_EQ(result.requested.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(result.requested.schema_encoding, "ros2msg");
  EXPECT_NE(result.requested.definition.find("---"), std::string::npos);
  EXPECT_TRUE(result.dependencies.empty());
}

TEST(LookupInterfaceDefinitionTest, RejectsMalformedType)
{
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("BatteryState")); }(), std::invalid_argument);
  EXPECT_THROW(
    []() { static_cast<void>(lookupInterfaceDefinitions("sensor_msgs/BatteryState")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("sensor_msgs/msg/")); }(), std::invalid_argument);
}

TEST(LookupInterfaceDefinitionTest, RejectsUnknownPackage)
{
  EXPECT_THROW(lookupInterfaceDefinitions("nonexistent_pkg/msg/Foo"), std::runtime_error);
}

TEST(LookupInterfaceDefinitionTest, RejectsUnknownMessage)
{
  EXPECT_THROW(lookupInterfaceDefinitions("std_msgs/msg/NonexistentMessage"), std::runtime_error);
}

}  // namespace

}  // namespace livekit_ros2_bridge
