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
#include "interface_definition_lookup.hpp"

namespace livekit_ros2_bridge
{

namespace
{

class ScopedLookupStateReset
{
public:
  ScopedLookupStateReset()
  {
    resetInterfaceDefinitionLookupStateForTest();
  }

  ~ScopedLookupStateReset()
  {
    resetInterfaceDefinitionLookupStateForTest();
  }
};

template <typename Fn>
std::string captureRuntimeError(Fn && fn)
{
  try {
    fn();
    ADD_FAILURE() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    return exc.what();
  }
  return "";
}

template <typename ValidateError>
void expectLookupFailureCachedUntilReset(const std::string & interface_type, ValidateError validate_error)
{
  SCOPED_TRACE(interface_type);

  ScopedLookupStateReset state_reset;

  int attempts = 0;
  const auto count_attempts = [&attempts](const std::string &) { ++attempts; };
  setInterfaceDefinitionLookupAttemptHookForTest(count_attempts);

  const auto lookup = [&interface_type]() { (void)lookupInterfaceDefinitions(interface_type); };

  const std::string first_error = captureRuntimeError(lookup);
  const std::string cached_error = captureRuntimeError(lookup);

  validate_error(first_error);
  EXPECT_EQ(cached_error, first_error);
  EXPECT_EQ(attempts, 1);

  resetInterfaceDefinitionLookupStateForTest();
  setInterfaceDefinitionLookupAttemptHookForTest(count_attempts);

  const std::string retried_error = captureRuntimeError(lookup);

  EXPECT_EQ(retried_error, first_error);
  EXPECT_EQ(attempts, 2);
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
  const auto & header = result[0];
  const auto & time = result[1];
  EXPECT_EQ(header.interface_type, "std_msgs/msg/Header");
  EXPECT_NE(header.definition.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_EQ(time.interface_type, "builtin_interfaces/msg/Time");
  EXPECT_NE(time.definition.find("int32 sec"), std::string::npos);
}

TEST(LookupInterfaceDefinitionTest, LooksUpTransitiveDependenciesWithoutDuplicates)
{
  const auto result = lookupInterfaceDefinitions("sensor_msgs/msg/BatteryState");

  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result.front().interface_type, "sensor_msgs/msg/BatteryState");
  std::set<std::string> dependency_types;
  for (auto it = result.begin() + 1; it != result.end(); ++it) {
    dependency_types.insert(it->interface_type);
  }
  const std::set<std::string> expected_dependencies = {
    "builtin_interfaces/msg/Time",
    "std_msgs/msg/Header",
  };
  EXPECT_EQ(dependency_types, expected_dependencies);
}

TEST(LookupInterfaceDefinitionTest, LooksUpPrimitiveOnlyServiceWithoutDependencies)
{
  const auto result = lookupInterfaceDefinitions("std_srvs/srv/SetBool");

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().interface_type, "std_srvs/srv/SetBool");
  EXPECT_NE(result.front().definition.find("---"), std::string::npos);
}

TEST(LookupInterfaceDefinitionTest, LooksUpServiceWithNestedMessageDependenciesInTraversalOrder)
{
  const auto result = lookupInterfaceDefinitions("sensor_msgs/srv/SetCameraInfo");

  ASSERT_EQ(result.size(), 5u);
  EXPECT_EQ(result[0].interface_type, "sensor_msgs/srv/SetCameraInfo");
  EXPECT_NE(result[0].definition.find("sensor_msgs/CameraInfo camera_info"), std::string::npos);
  EXPECT_NE(result[0].definition.find("---"), std::string::npos);

  EXPECT_EQ(result[1].interface_type, "sensor_msgs/msg/CameraInfo");
  EXPECT_EQ(result[2].interface_type, "std_msgs/msg/Header");
  EXPECT_EQ(result[3].interface_type, "builtin_interfaces/msg/Time");
  EXPECT_EQ(result[4].interface_type, "sensor_msgs/msg/RegionOfInterest");
}

TEST(LookupInterfaceDefinitionTest, RejectsMalformedType)
{
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("BatteryState")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("sensor_msgs/msg/")); }(), std::invalid_argument);
  EXPECT_THROW(
    []() { static_cast<void>(lookupInterfaceDefinitions("std_msgs/msg/String/Extra")); }(), std::invalid_argument);
  EXPECT_THROW(
    []() { static_cast<void>(lookupInterfaceDefinitions("std_msgs/topic/String")); }(), std::invalid_argument);
}

TEST(LookupInterfaceDefinitionTest, CachesLookupFailuresUntilReset)
{
  expectLookupFailureCachedUntilReset("nonexistent_pkg/msg/Foo", [](const std::string & first_error) {
    EXPECT_EQ(first_error, "Package 'nonexistent_pkg' not found in ament index");
  });

  expectLookupFailureCachedUntilReset("std_msgs/msg/NonexistentMessage", [](const std::string & first_error) {
    EXPECT_EQ(first_error.find("Cannot open interface definition file: "), 0u);
    EXPECT_NE(first_error.find("/msg/NonexistentMessage.msg"), std::string::npos);
  });
}

}  // namespace

}  // namespace livekit_ros2_bridge
