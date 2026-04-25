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
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{
namespace
{

std::string expandName(std::string_view name)
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
  EXPECT_EQ(normalizeRosResourceName("camera"), expandName("camera"));
  EXPECT_EQ(normalizeRosResourceName("  /camera/front/image  "), expandName("/camera/front/image"));
  EXPECT_EQ(normalizeRosResourceName("{node}/image"), expandName("{node}/image"));
}

TEST(NormalizeRosResourceNameTest, RosValidationFailuresReturnEmpty)
{
  constexpr char kInvalidName[] = "/camera///front/image/";
  EXPECT_THROW(expandName(kInvalidName), rclcpp::exceptions::NameValidationError);
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

TEST(RosResourcePatternTest, ParsesExactAndSubtreePatternsWithRosNameNormalization)
{
  const auto exact = RosResourcePattern::parse(" camera/front ");
  ASSERT_TRUE(exact.has_value());
  EXPECT_EQ(exact->canonical(), "/camera/front");
  EXPECT_EQ(exact->kind(), RosResourcePattern::Kind::Exact);
  EXPECT_TRUE(exact->matches("/camera/front"));
  EXPECT_FALSE(exact->matches("/camera/front/image"));

  const auto subtree = RosResourcePattern::parse(" camera/front/* ");
  ASSERT_TRUE(subtree.has_value());
  EXPECT_EQ(subtree->canonical(), "/camera/front/*");
  EXPECT_EQ(subtree->kind(), RosResourcePattern::Kind::Subtree);
  EXPECT_FALSE(subtree->matches("/camera/front"));
  EXPECT_TRUE(subtree->matches("/camera/front/image"));
}

TEST(RosResourcePatternTest, ParsesCatchAllShorthandsAsRootSubtreeWildcard)
{
  const auto star = RosResourcePattern::parse(" * ");
  ASSERT_TRUE(star.has_value());
  EXPECT_EQ(star->canonical(), "/*");
  EXPECT_TRUE(star->matches("/"));
  EXPECT_TRUE(star->matches("/camera/front/image"));

  const auto root_subtree = RosResourcePattern::parse(" /* ");
  ASSERT_TRUE(root_subtree.has_value());
  EXPECT_EQ(root_subtree->canonical(), "/*");
  EXPECT_TRUE(root_subtree->matches("/"));
  EXPECT_TRUE(root_subtree->matches("/camera/front/image"));
}

TEST(RosResourcePatternTest, RejectsBlankInvalidAndRootExactPatterns)
{
  EXPECT_FALSE(RosResourcePattern::parse(" \t\n ").has_value());
  EXPECT_FALSE(RosResourcePattern::parse("/camera///front/image/").has_value());
  EXPECT_FALSE(RosResourcePattern::parse("/").has_value());
}

TEST(RosResourcePatternTest, BestMatchUsesLongestMatchAndKeepsFirstDeclaredTie)
{
  struct Rule
  {
    RosResourcePattern pattern;
    std::string id;
  };

  const std::vector<Rule> rules{
    {RosResourcePattern::fromCanonical("/*"), "root"},
    {RosResourcePattern::fromCanonical("/camera/*"), "camera"},
    {RosResourcePattern::fromCanonical("/camera/front/*"), "front_first"},
    {RosResourcePattern::fromCanonical("/camera/front/*"), "front_second"},
  };

  const auto * match = findBestRosResourcePatternMatch(rules, "/camera/front/image", &Rule::pattern);
  ASSERT_NE(match, nullptr);
  EXPECT_EQ(match->id, "front_first");
}

}  // namespace livekit_ros2_bridge
