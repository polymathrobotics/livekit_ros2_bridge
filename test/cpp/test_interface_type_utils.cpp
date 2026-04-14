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

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "utils/interface_type_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::string requireSingleInterfaceTypeError(
  const std::map<std::string, std::vector<std::string>> & names, const std::string & name, const char * resource_kind)
{
  try {
    static_cast<void>(requireSingleInterfaceType(names, name, resource_kind));
    ADD_FAILURE() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    return error.what();
  } catch (const std::exception & error) {
    ADD_FAILURE() << "Expected std::invalid_argument, got: " << error.what();
  }
  return {};
}

TEST(RequireSingleInterfaceTypeTest, ReturnsTheOnlyAdvertisedType)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {"bar/msg/Baz"}},
  };
  EXPECT_EQ(requireSingleInterfaceType(names, "/foo", "topic"), "bar/msg/Baz");
}

TEST(RequireSingleInterfaceTypeTest, RejectsMissingOrUnusableTypeSetsAsNoTypesFound)
{
  const std::string expected_error = "No ROS types found for topic '/foo'.";

  EXPECT_EQ(
    requireSingleInterfaceTypeError(std::map<std::string, std::vector<std::string>>{}, "/foo", "topic"),
    expected_error);
  // A graph entry with only empty strings is not a usable ROS interface type.
  EXPECT_EQ(
    requireSingleInterfaceTypeError(std::map<std::string, std::vector<std::string>>{{"/foo", {""}}}, "/foo", "topic"),
    expected_error);
}

TEST(RequireSingleInterfaceTypeTest, RejectsAmbiguousTypeSetsAsMultipleTypesFound)
{
  EXPECT_EQ(
    requireSingleInterfaceTypeError(
      std::map<std::string, std::vector<std::string>>{{"/foo", {"a/msg/A", "b/msg/B"}}}, "/foo", "topic"),
    "Multiple ROS types found for topic '/foo'.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
