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
#include "ros_interfaces/graph_lookup.hpp"

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

std::string requireSingleTypeError(
  const std::map<std::string, std::vector<std::string>> & graph, const std::string & resource, const char * kind)
{
  try {
    static_cast<void>(requireSingleType(graph, resource, kind));
    ADD_FAILURE() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    return error.what();
  } catch (const std::exception & error) {
    ADD_FAILURE() << "Expected std::invalid_argument, got: " << error.what();
  }
  return {};
}

TEST(RequireSingleTypeTest, ReturnsTheOnlyAdvertisedType)
{
  const std::map<std::string, std::vector<std::string>> graph{
    {"/foo", {"bar/msg/Baz"}},
  };
  EXPECT_EQ(requireSingleType(graph, "/foo", "topic"), "bar/msg/Baz");
}

TEST(RequireSingleTypeTest, RejectsMissingTypeSetsAsNoTypesFound)
{
  EXPECT_EQ(requireSingleTypeError({}, "/foo", "topic"), "No ROS types found for topic '/foo'.");
}

TEST(RequireSingleTypeTest, RejectsAmbiguousTypeSetsAsMultipleTypesFound)
{
  EXPECT_EQ(
    requireSingleTypeError({{"/foo", {"a/msg/A", "b/msg/B"}}}, "/foo", "topic"),
    "Multiple ROS types found for topic '/foo'.");
}

}  // namespace

}  // namespace livekit_ros2_bridge::ros_interfaces
