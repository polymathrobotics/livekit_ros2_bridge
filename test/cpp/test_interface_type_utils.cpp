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
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(RequireSingleInterfaceTypeTest, ReturnsTheOnlyType)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {"bar/msg/Baz"}},
  };
  EXPECT_EQ(requireSingleInterfaceType(names, "/foo", "topic"), "bar/msg/Baz");
}

TEST(RequireSingleInterfaceTypeTest, ThrowsWhenNameNotFound)
{
  std::map<std::string, std::vector<std::string>> names;
  EXPECT_THROW(requireSingleInterfaceType(names, "/foo", "topic"), std::invalid_argument);
}

TEST(RequireSingleInterfaceTypeTest, ThrowsWhenMultipleTypes)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {"a/msg/A", "b/msg/B"}},
  };
  EXPECT_THROW(requireSingleInterfaceType(names, "/foo", "topic"), std::invalid_argument);
}

TEST(RequireSingleInterfaceTypeTest, ThrowsWhenTypesVectorEmpty)
{
  std::map<std::string, std::vector<std::string>> names{
    {"/foo", {}},
  };
  EXPECT_THROW(requireSingleInterfaceType(names, "/foo", "service"), std::invalid_argument);
}

TEST(ClassifyRosVideoInterfaceTypeTest, RawImageTypeIsVideo)
{
  const auto interface_classification = classifyRosVideoInterfaceType(kImageInterfaceType);
  ASSERT_TRUE(interface_classification.has_value());
  EXPECT_EQ(interface_classification->ingest_mode, kRawImageIngestMode);
}

TEST(ClassifyRosVideoInterfaceTypeTest, CompressedImageTypeIsVideo)
{
  const auto interface_classification = classifyRosVideoInterfaceType(kCompressedImageInterfaceType);
  ASSERT_TRUE(interface_classification.has_value());
  EXPECT_EQ(interface_classification->ingest_mode, kCompressedImageIngestMode);
}

TEST(ClassifyRosVideoInterfaceTypeTest, StringTypeIsNotVideo)
{
  EXPECT_FALSE(classifyRosVideoInterfaceType("std_msgs/msg/String").has_value());
}

TEST(ClassifyRosVideoInterfaceTypeTest, EmptyStringIsNotVideo)
{
  EXPECT_FALSE(classifyRosVideoInterfaceType("").has_value());
}

TEST(ClassifyRosVideoInterfaceTypeTest, PartialImageTypeIsNotVideo)
{
  EXPECT_FALSE(classifyRosVideoInterfaceType("sensor_msgs/msg/Imag").has_value());
}

TEST(ClassifyRosVideoInterfaceTypeTest, CaseVariantIsNotVideo)
{
  EXPECT_FALSE(classifyRosVideoInterfaceType("sensor_msgs/msg/image").has_value());
}

TEST(ClassifyRosVideoInterfaceTypeTest, ExtraWhitespaceIsNotVideo)
{
  EXPECT_FALSE(classifyRosVideoInterfaceType(" sensor_msgs/msg/Image").has_value());
  EXPECT_FALSE(classifyRosVideoInterfaceType("sensor_msgs/msg/Image ").has_value());
}

}  // namespace

}  // namespace livekit_ros2_bridge
