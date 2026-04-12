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

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/interface_payloads.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(InterfacePayloadsTest, ParsesValidRequest)
{
  const auto types = parseRequestedInterfaceTypes(R"({"interface_types":["sensor_msgs/msg/BatteryState"]})");
  ASSERT_EQ(types.size(), 1u);
  EXPECT_EQ(types[0], "sensor_msgs/msg/BatteryState");
}

TEST(InterfacePayloadsTest, ParsesMultipleTypes)
{
  const auto types =
    parseRequestedInterfaceTypes(R"({"interface_types":["sensor_msgs/msg/BatteryState","std_msgs/msg/String"]})");
  ASSERT_EQ(types.size(), 2u);
  EXPECT_EQ(types[0], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(types[1], "std_msgs/msg/String");
}

TEST(InterfacePayloadsTest, TrimsTypeWhitespace)
{
  const auto types = parseRequestedInterfaceTypes(R"({"interface_types":["  sensor_msgs/msg/BatteryState  "]})");
  ASSERT_EQ(types.size(), 1u);
  EXPECT_EQ(types[0], "sensor_msgs/msg/BatteryState");
}

TEST(InterfacePayloadsTest, RejectsMissingInterfaceTypes)
{
  EXPECT_THROW(parseRequestedInterfaceTypes(R"({})"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsEmptyArray)
{
  EXPECT_THROW(parseRequestedInterfaceTypes(R"({"interface_types":[]})"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsEmptyStringEntry)
{
  EXPECT_THROW(parseRequestedInterfaceTypes(R"({"interface_types":[""]})"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsNonArrayInterfaceTypes)
{
  EXPECT_THROW(parseRequestedInterfaceTypes(R"({"interface_types":"not_an_array"})"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsNonStringEntry)
{
  EXPECT_THROW(parseRequestedInterfaceTypes(R"({"interface_types":[42]})"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsInvalidJson)
{
  EXPECT_THROW(parseRequestedInterfaceTypes("{"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, RejectsNonObject)
{
  EXPECT_THROW(parseRequestedInterfaceTypes("[1,2]"), std::invalid_argument);
}

TEST(InterfacePayloadsTest, SerializesResponse)
{
  std::vector<InterfaceDefinition> interfaces = {
    {"sensor_msgs/msg/BatteryState", "ros2msg", "float32 voltage\n"},
    {"std_msgs/msg/Header", "ros2msg", "builtin_interfaces/Time stamp\nstring frame_id\n"},
  };

  const auto serialized = serializeInterfacesResponse(interfaces);
  const auto body = nlohmann::json::parse(serialized);

  ASSERT_TRUE(body.contains("interfaces"));
  ASSERT_TRUE(body["interfaces"].is_array());
  ASSERT_EQ(body["interfaces"].size(), 2u);

  EXPECT_EQ(body["interfaces"][0]["interface_type"].get<std::string>(), "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(body["interfaces"][0]["format"].get<std::string>(), "ros2msg");
  EXPECT_EQ(body["interfaces"][0]["definition"].get<std::string>(), "float32 voltage\n");

  EXPECT_EQ(body["interfaces"][1]["interface_type"].get<std::string>(), "std_msgs/msg/Header");
  EXPECT_EQ(body["interfaces"][1]["format"].get<std::string>(), "ros2msg");
  EXPECT_EQ(body["interfaces"][1]["definition"].get<std::string>(), "builtin_interfaces/Time stamp\nstring frame_id\n");
}

TEST(InterfacePayloadsTest, SerializesEmptyVector)
{
  std::vector<InterfaceDefinition> interfaces;

  const auto serialized = serializeInterfacesResponse(interfaces);
  const auto body = nlohmann::json::parse(serialized);

  ASSERT_TRUE(body.contains("interfaces"));
  ASSERT_TRUE(body["interfaces"].is_array());
  EXPECT_TRUE(body["interfaces"].empty());
}

}  // namespace

}  // namespace livekit_ros2_bridge
