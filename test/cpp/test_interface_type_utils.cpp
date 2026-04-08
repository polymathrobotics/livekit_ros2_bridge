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
#include "utils/interface_types.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(RequireUniqueInterfaceTypeTest, ReturnsTheOnlyType)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {"bar/msg/Baz"}},
  };
  EXPECT_EQ(requireUniqueInterfaceType(names, "/foo", "topic"), "bar/msg/Baz");
}

TEST(RequireUniqueInterfaceTypeTest, ThrowsWhenNameNotFound)
{
  std::map<std::string, std::vector<std::string>> names;
  EXPECT_THROW(requireUniqueInterfaceType(names, "/foo", "topic"), std::invalid_argument);
}

TEST(RequireUniqueInterfaceTypeTest, ThrowsWhenMultipleTypes)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {"a/msg/A", "b/msg/B"}},
  };
  EXPECT_THROW(requireUniqueInterfaceType(names, "/foo", "topic"), std::invalid_argument);
}

TEST(RequireUniqueInterfaceTypeTest, ThrowsWhenTypesVectorEmpty)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {}},
  };
  EXPECT_THROW(requireUniqueInterfaceType(names, "/foo", "service"), std::invalid_argument);
}

TEST(IsSupportedVideoInterfaceTypeTest, RawImageTypeIsVideo)
{
  EXPECT_TRUE(isSupportedVideoInterfaceType("sensor_msgs/msg/Image"));
}

TEST(IsSupportedVideoInterfaceTypeTest, CompressedImageTypeIsVideo)
{
  EXPECT_TRUE(isSupportedVideoInterfaceType("sensor_msgs/msg/CompressedImage"));
}

TEST(IsSupportedVideoInterfaceTypeTest, StringTypeIsNotVideo)
{
  EXPECT_FALSE(isSupportedVideoInterfaceType("std_msgs/msg/String"));
}

TEST(IsSupportedVideoInterfaceTypeTest, EmptyStringIsNotVideo)
{
  EXPECT_FALSE(isSupportedVideoInterfaceType(""));
}

TEST(IsSupportedVideoInterfaceTypeTest, PartialImageTypeIsNotVideo)
{
  EXPECT_FALSE(isSupportedVideoInterfaceType("sensor_msgs/msg/Imag"));
}

TEST(IsSupportedVideoInterfaceTypeTest, CaseVariantIsNotVideo)
{
  EXPECT_FALSE(isSupportedVideoInterfaceType("sensor_msgs/msg/image"));
}

TEST(IsSupportedVideoInterfaceTypeTest, ExtraWhitespaceIsNotVideo)
{
  EXPECT_FALSE(isSupportedVideoInterfaceType(" sensor_msgs/msg/Image"));
  EXPECT_FALSE(isSupportedVideoInterfaceType("sensor_msgs/msg/Image "));
}

}  // namespace

}  // namespace livekit_ros2_bridge
