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

class ScopedInterfaceLookupReset
{
public:
  // lookupInterfaceDefinitions keeps process-wide negative-cache and hook state, so each test
  // helper brackets its assertions with a full reset.
  ScopedInterfaceLookupReset()
  {
    resetInterfaceLookupForTest();
  }

  ~ScopedInterfaceLookupReset()
  {
    resetInterfaceLookupForTest();
  }
};

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

template <typename Exception, typename ValidateError>
void expectFailureCachedUntilReset(
  const std::string & interface_type, const char * type_name, ValidateError validate_error)
{
  // The attempt hook lets the test distinguish a cache hit from a fresh lookup without reaching
  // into the cache implementation itself.
  SCOPED_TRACE(interface_type);

  ScopedInterfaceLookupReset reset;

  int attempts = 0;
  const auto count_attempts = [&attempts](const std::string &) { ++attempts; };
  setInterfaceLookupAttemptHookForTest(count_attempts);

  const auto run_lookup = [&interface_type]() { (void)lookupInterfaceDefinitions(interface_type); };

  const std::string first_error = captureException<Exception>(run_lookup, type_name);
  const std::string cached_error = captureException<Exception>(run_lookup, type_name);

  validate_error(first_error);
  EXPECT_EQ(cached_error, first_error);
  EXPECT_EQ(attempts, 1);

  resetInterfaceLookupForTest();
  setInterfaceLookupAttemptHookForTest(count_attempts);

  const std::string retried_error = captureException<Exception>(run_lookup, type_name);

  EXPECT_EQ(retried_error, first_error);
  EXPECT_EQ(attempts, 2);
}

TEST(InterfaceDefinitionLookupTest, LooksUpSimpleMessageWithoutDependencies)
{
  const auto definitions = lookupInterfaceDefinitions("std_msgs/msg/String");

  ASSERT_EQ(definitions.size(), 1u);
  EXPECT_EQ(definitions.front().interface_type, "std_msgs/msg/String");
  EXPECT_EQ(definitions.front().format, "ros2msg");
  EXPECT_NE(definitions.front().definition.find("string data"), std::string::npos);
}

TEST(InterfaceDefinitionLookupTest, LooksUpMessageWithDirectDependencies)
{
  const auto definitions = lookupInterfaceDefinitions("std_msgs/msg/Header");

  ASSERT_EQ(definitions.size(), 2u);
  const auto & header = definitions[0];
  const auto & time = definitions[1];
  EXPECT_EQ(header.interface_type, "std_msgs/msg/Header");
  EXPECT_NE(header.definition.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_EQ(time.interface_type, "builtin_interfaces/msg/Time");
  EXPECT_NE(time.definition.find("int32 sec"), std::string::npos);
}

TEST(InterfaceDefinitionLookupTest, LooksUpTransitiveDependenciesWithoutDuplicates)
{
  const auto definitions = lookupInterfaceDefinitions("sensor_msgs/msg/BatteryState");

  ASSERT_EQ(definitions.size(), 3u);
  EXPECT_EQ(definitions.front().interface_type, "sensor_msgs/msg/BatteryState");
  std::set<std::string> dependency_types;
  for (auto it = definitions.begin() + 1; it != definitions.end(); ++it) {
    dependency_types.insert(it->interface_type);
  }
  const std::set<std::string> expected_dependencies = {
    "builtin_interfaces/msg/Time",
    "std_msgs/msg/Header",
  };
  EXPECT_EQ(dependency_types, expected_dependencies);
}

TEST(InterfaceDefinitionLookupTest, LooksUpPrimitiveOnlyServiceWithoutDependencies)
{
  const auto definitions = lookupInterfaceDefinitions("std_srvs/srv/SetBool");

  ASSERT_EQ(definitions.size(), 1u);
  EXPECT_EQ(definitions.front().interface_type, "std_srvs/srv/SetBool");
  EXPECT_NE(definitions.front().definition.find("---"), std::string::npos);
}

TEST(InterfaceDefinitionLookupTest, LooksUpServiceWithNestedMessageDependenciesInTraversalOrder)
{
  const auto definitions = lookupInterfaceDefinitions("sensor_msgs/srv/SetCameraInfo");

  ASSERT_EQ(definitions.size(), 5u);
  EXPECT_EQ(definitions[0].interface_type, "sensor_msgs/srv/SetCameraInfo");
  EXPECT_NE(definitions[0].definition.find("sensor_msgs/CameraInfo camera_info"), std::string::npos);
  EXPECT_NE(definitions[0].definition.find("---"), std::string::npos);

  EXPECT_EQ(definitions[1].interface_type, "sensor_msgs/msg/CameraInfo");
  EXPECT_EQ(definitions[2].interface_type, "std_msgs/msg/Header");
  EXPECT_EQ(definitions[3].interface_type, "builtin_interfaces/msg/Time");
  EXPECT_EQ(definitions[4].interface_type, "sensor_msgs/msg/RegionOfInterest");
}

TEST(InterfaceDefinitionLookupTest, RejectsMalformedType)
{
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("BatteryState")); }(), std::invalid_argument);
  EXPECT_THROW([]() { static_cast<void>(lookupInterfaceDefinitions("sensor_msgs/msg/")); }(), std::invalid_argument);
  EXPECT_THROW(
    []() { static_cast<void>(lookupInterfaceDefinitions("std_msgs/msg/String/Extra")); }(), std::invalid_argument);
  EXPECT_THROW(
    []() { static_cast<void>(lookupInterfaceDefinitions("std_msgs/topic/String")); }(), std::invalid_argument);
}

TEST(InterfaceDefinitionLookupTest, CachesLookupFailuresUntilReset)
{
  expectFailureCachedUntilReset<std::runtime_error>(
    "nonexistent_pkg/msg/Foo", "std::runtime_error", [](const std::string & first_error) {
      EXPECT_EQ(first_error, "Package 'nonexistent_pkg' not found in ament index");
    });

  expectFailureCachedUntilReset<std::runtime_error>(
    "std_msgs/msg/NonexistentMessage", "std::runtime_error", [](const std::string & first_error) {
      EXPECT_EQ(first_error.find("Cannot open interface definition file: "), 0u);
      EXPECT_NE(first_error.find("/msg/NonexistentMessage.msg"), std::string::npos);
    });
}

TEST(InterfaceDefinitionLookupTest, CachesMalformedTypeFailuresUntilReset)
{
  expectFailureCachedUntilReset<std::invalid_argument>(
    "BatteryState", "std::invalid_argument", [](const std::string & first_error) {
      EXPECT_EQ(first_error, "Invalid ROS interface type 'BatteryState': expected package/kind/Name");
    });
}

}  // namespace

}  // namespace livekit_ros2_bridge
