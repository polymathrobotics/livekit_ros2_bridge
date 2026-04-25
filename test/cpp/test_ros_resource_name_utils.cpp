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

#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{
namespace
{

std::string expandRosResourceName(std::string_view name)
{
  return rclcpp::expand_topic_or_service_name(std::string{name}, "livekit_ros2_bridge_resource_name", "/");
}

}  // namespace

TEST(NormalizeRosResourceNameTest, EmptyAndWhitespaceOnlyInputsReturnEmpty)
{
  EXPECT_EQ(normalizeRosResourceName(""), "");
  EXPECT_EQ(normalizeRosResourceName("\t\n"), "");
}

TEST(NormalizeRosResourceNameTest, TrimsAndDelegatesExpansionToRclcpp)
{
  EXPECT_EQ(normalizeRosResourceName("camera"), expandRosResourceName("camera"));
  EXPECT_EQ(normalizeRosResourceName("  /camera/front/image  "), expandRosResourceName("/camera/front/image"));
  EXPECT_EQ(normalizeRosResourceName("{node}/image"), expandRosResourceName("{node}/image"));
}

TEST(NormalizeRosResourceNameTest, RosValidationFailuresReturnEmpty)
{
  constexpr char kInvalidName[] = "/camera///front/image/";
  EXPECT_THROW(expandRosResourceName(kInvalidName), rclcpp::exceptions::NameValidationError);
  EXPECT_EQ(normalizeRosResourceName(kInvalidName), "");
}

TEST(RosResourceMatchesPatternTest, ExactPatternsMatchOnlyIdenticalNames)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/camera", "/camera"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera/image", "/camera"));
  EXPECT_TRUE(rosResourceMatchesPattern("/", "/"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera", "/"));
}

TEST(RosResourceMatchesPatternTest, SubtreePatternMatchesDescendantsButNotPrefixNeighbors)
{
  EXPECT_FALSE(rosResourceMatchesPattern("/camera", "/camera/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/front/image", "/camera/*"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera_front/image", "/camera/*"));
}

TEST(RosResourceMatchesPatternTest, RootSubtreeWildcardMatchesRootAndDescendants)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/", "/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera", "/*"));
}

TEST(RosResourceMatchesPatternTest, OnlyTerminalSlashStarActsAsSubtreeWildcard)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/*/image", "/camera/*/image"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera/front/image", "/camera/*/image"));
}

}  // namespace livekit_ros2_bridge
