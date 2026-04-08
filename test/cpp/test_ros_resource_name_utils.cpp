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

TEST(NormalizeRosResourceNameTest, EmptyAndWhitespaceInputReturnsEmpty)
{
  EXPECT_EQ(normalizeRosResourceName(""), "");
  EXPECT_EQ(normalizeRosResourceName("   "), "");
  EXPECT_EQ(normalizeRosResourceName("\t\n"), "");
}

TEST(NormalizeRosResourceNameTest, NormalizesCommonTopicInputsToCanonicalForm)
{
  EXPECT_EQ(normalizeRosResourceName("camera"), "/camera");
  EXPECT_EQ(normalizeRosResourceName(" camera//image/ "), "/camera/image");
  EXPECT_EQ(normalizeRosResourceName("  /camera///front/image  "), "/camera/front/image");
}

TEST(NormalizeRosResourceNameTest, RootSlashAloneIsPreserved)
{
  EXPECT_EQ(normalizeRosResourceName("/"), "/");
}

TEST(RosResourceMatchesPatternTest, ExactPatternMatchesOnlyIdenticalName)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/camera", "/camera"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera/image", "/camera"));
}

TEST(RosResourceMatchesPatternTest, SubtreePatternMatchesDescendantsButNotPrefixNeighbors)
{
  EXPECT_FALSE(rosResourceMatchesPattern("/camera", "/camera/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/image", "/camera/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/front/image", "/camera/*"));
  EXPECT_FALSE(rosResourceMatchesPattern("/camera_front/image", "/camera/*"));
}

TEST(RosResourceMatchesPatternTest, RootSubtreePatternMatchesAllAbsolutePaths)
{
  EXPECT_TRUE(rosResourceMatchesPattern("/", "/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera", "/*"));
  EXPECT_TRUE(rosResourceMatchesPattern("/camera/image", "/*"));
}

}  // namespace livekit_ros2_bridge
