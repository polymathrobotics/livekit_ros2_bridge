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

#include <functional>
#include <set>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "interface_definition_lookup.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::set<std::string> dependencyTypes(const std::vector<InterfaceDefinition> & definitions)
{
  std::set<std::string> types;
  const auto first_dependency = definitions.begin() + 1;
  for (auto it = first_dependency; it != definitions.end(); ++it) {
    types.insert(it->interface_type);
  }
  return types;
}

std::string expectRuntimeErrorMessage(const std::function<void()> & action)
{
  try {
    action();
    ADD_FAILURE() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    return exc.what();
  } catch (...) {
    ADD_FAILURE() << "Expected std::runtime_error";
  }
  return "";
}

TEST(LookupInterfaceDefinitionTest, LooksUpSimpleMessageWithoutDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_msgs/msg/String");

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().interface_type, "std_msgs/msg/String");
  EXPECT_EQ(result.front().format, "ros2msg");
  EXPECT_NE(result.front().definition.find("string data"), std::string::npos);
}

TEST(LookupInterfaceDefinitionTest, LooksUpMessageWithDirectDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_msgs/msg/Header");

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].interface_type, "std_msgs/msg/Header");
  EXPECT_NE(result[0].definition.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_EQ(result[1].interface_type, "builtin_interfaces/msg/Time");
  EXPECT_EQ(result[1].format, "ros2msg");
  EXPECT_NE(result[1].definition.find("int32 sec"), std::string::npos);
}

TEST(LookupInterfaceDefinitionTest, LooksUpTransitiveDependenciesWithoutDuplicates)
{
  const auto result = lookupInterfaceDefinitions("sensor_msgs/msg/BatteryState");

  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result.front().interface_type, "sensor_msgs/msg/BatteryState");
  const std::set<std::string> expected_dependencies = {
    "builtin_interfaces/msg/Time",
    "std_msgs/msg/Header",
  };
  EXPECT_EQ(dependencyTypes(result), expected_dependencies);
  EXPECT_EQ(result.size() - 1U, expected_dependencies.size());
}

TEST(LookupInterfaceDefinitionTest, LooksUpPrimitiveOnlyServiceWithoutDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_srvs/srv/SetBool");

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(result.front().format, "ros2msg");
  EXPECT_NE(result.front().definition.find("---"), std::string::npos);
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

TEST(LookupInterfaceDefinitionTest, CachesInvalidLookupFailures)
{
  resetInterfaceDefinitionLookupStateForTest();

  int attempts = 0;
  setInterfaceDefinitionLookupAttemptHookForTest([&attempts](const std::string & interface_type) {
    ++attempts;
    EXPECT_EQ(interface_type, "nonexistent_pkg/msg/Foo");
  });

  const std::string first_error =
    expectRuntimeErrorMessage([]() { static_cast<void>(lookupInterfaceDefinitions("nonexistent_pkg/msg/Foo")); });
  const std::string second_error =
    expectRuntimeErrorMessage([]() { static_cast<void>(lookupInterfaceDefinitions("nonexistent_pkg/msg/Foo")); });

  EXPECT_EQ(attempts, 1);
  EXPECT_EQ(second_error, first_error);

  resetInterfaceDefinitionLookupStateForTest();
}

TEST(LookupInterfaceDefinitionTest, RejectsUnknownMessage)
{
  EXPECT_THROW(lookupInterfaceDefinitions("std_msgs/msg/NonexistentMessage"), std::runtime_error);
}

}  // namespace

}  // namespace livekit_ros2_bridge
