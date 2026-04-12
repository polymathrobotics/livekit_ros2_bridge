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

#include "gtest/gtest.h"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

TEST(NormalizeRosResourceNameTest, EmptyAndWhitespaceOnlyInputsReturnEmpty)
{
  EXPECT_EQ(normalizeRosResourceName(""), "");
  EXPECT_EQ(normalizeRosResourceName("\t\n"), "");
}

TEST(NormalizeRosResourceNameTest, NormalizesCommonInputsToCanonicalAbsoluteNames)
{
  EXPECT_EQ(normalizeRosResourceName("camera"), "/camera");
  EXPECT_EQ(normalizeRosResourceName("  /camera///front/image/  "), "/camera/front/image");
  EXPECT_EQ(normalizeRosResourceName(" blocked//tree//* "), "/blocked/tree/*");
}

TEST(NormalizeRosResourceNameTest, RootInputsNormalizeToRootName)
{
  EXPECT_EQ(normalizeRosResourceName("  ////  "), "/");
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
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/image", "/camera/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/front/image", "/camera/*"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera_front/image", "/camera/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/", "/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera", "/*"));
}

TEST(RosResourceMatchesPatternTest, OnlyTerminalSlashStarActsAsSubtreeWildcard)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/*/image", "/camera/*/image"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera/front/image", "/camera/*/image"));
}

}  // namespace livekit_ros2_bridge
