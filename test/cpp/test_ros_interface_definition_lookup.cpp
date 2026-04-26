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
#include <string>

#include "gtest/gtest.h"
#include "ros_interfaces/definition_lookup.hpp"

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

template <typename Exception, typename Fn>
std::string captureException(Fn && fn, const char * type_name)
{
  try {
    fn();
    ADD_FAILURE() << "Expected " << type_name;
  } catch (const Exception & exc) {
    return exc.what();
  }
  return "";
}

TEST(DefinitionLookupTest, LooksUpSimpleMessageWithoutDependencies)
{
  const auto definitions = lookupDefinitions("std_msgs/msg/String");

  ASSERT_EQ(definitions.size(), 1u);
  EXPECT_EQ(definitions.front().type, "std_msgs/msg/String");
  EXPECT_NE(definitions.front().body.find("string data"), std::string::npos);
}

TEST(DefinitionLookupTest, LooksUpMessageWithDirectDependencies)
{
  const auto definitions = lookupDefinitions("std_msgs/msg/Header");

  ASSERT_EQ(definitions.size(), 2u);
  const auto & header = definitions[0];
  const auto & time = definitions[1];
  EXPECT_EQ(header.type, "std_msgs/msg/Header");
  EXPECT_NE(header.body.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_EQ(time.type, "builtin_interfaces/msg/Time");
  EXPECT_NE(time.body.find("int32 sec"), std::string::npos);
}

TEST(DefinitionLookupTest, LooksUpTransitiveDependenciesWithoutDuplicates)
{
  const auto definitions = lookupDefinitions("sensor_msgs/msg/BatteryState");

  ASSERT_EQ(definitions.size(), 3u);
  EXPECT_EQ(definitions.front().type, "sensor_msgs/msg/BatteryState");
  std::set<std::string> types;
  for (auto it = definitions.begin() + 1; it != definitions.end(); ++it) {
    types.insert(it->type);
  }
  const std::set<std::string> expected = {
    "builtin_interfaces/msg/Time",
    "std_msgs/msg/Header",
  };
  EXPECT_EQ(types, expected);
}

TEST(DefinitionLookupTest, LooksUpPrimitiveOnlyServiceWithoutDependencies)
{
  const auto definitions = lookupDefinitions("std_srvs/srv/SetBool");

  ASSERT_EQ(definitions.size(), 1u);
  EXPECT_EQ(definitions.front().type, "std_srvs/srv/SetBool");
  EXPECT_NE(definitions.front().body.find("---"), std::string::npos);
}

TEST(DefinitionLookupTest, LooksUpServiceWithNestedMessageDependenciesInTraversalOrder)
{
  const auto definitions = lookupDefinitions("sensor_msgs/srv/SetCameraInfo");

  ASSERT_EQ(definitions.size(), 5u);
  EXPECT_EQ(definitions[0].type, "sensor_msgs/srv/SetCameraInfo");
  EXPECT_NE(definitions[0].body.find("sensor_msgs/CameraInfo camera_info"), std::string::npos);
  EXPECT_NE(definitions[0].body.find("---"), std::string::npos);

  EXPECT_EQ(definitions[1].type, "sensor_msgs/msg/CameraInfo");
  EXPECT_EQ(definitions[2].type, "std_msgs/msg/Header");
  EXPECT_EQ(definitions[3].type, "builtin_interfaces/msg/Time");
  EXPECT_EQ(definitions[4].type, "sensor_msgs/msg/RegionOfInterest");
}

TEST(DefinitionLookupTest, RejectsMalformedTypeShapeAndKind)
{
  EXPECT_THROW([]() { static_cast<void>(lookupDefinitions("sensor_msgs/msg/")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupDefinitions("std_msgs/msg/String/Extra")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupDefinitions("std_msgs/topic/String")); }(), std::invalid_argument);
}

TEST(DefinitionLookupTest, ReportsLookupFailures)
{
  const auto missing_package = []() { (void)lookupDefinitions("nonexistent_pkg/msg/Foo"); };
  EXPECT_EQ(
    captureException<std::runtime_error>(missing_package, "std::runtime_error"),
    "Package 'nonexistent_pkg' not found in ament index");

  const auto missing_definition = []() { (void)lookupDefinitions("std_msgs/msg/NonexistentMessage"); };
  const std::string error = captureException<std::runtime_error>(missing_definition, "std::runtime_error");
  EXPECT_EQ(error.find("Cannot open interface definition file: "), 0u);
  EXPECT_NE(error.find("/msg/NonexistentMessage.msg"), std::string::npos);
}

TEST(DefinitionLookupTest, ReportsMalformedTypeFailures)
{
  const auto malformed_type = []() { (void)lookupDefinitions("BatteryState"); };
  EXPECT_EQ(
    captureException<std::invalid_argument>(malformed_type, "std::invalid_argument"),
    "Invalid ROS interface type 'BatteryState': expected package/kind/Name");
}

}  // namespace

}  // namespace livekit_ros2_bridge::ros_interfaces
