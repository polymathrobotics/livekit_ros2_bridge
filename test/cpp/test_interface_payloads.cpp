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

#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/interfaces_json.hpp"
#include "protocol_test_support.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using test_support::expectInvalidArgument;

TEST(InterfacePayloadsTest, ParsesTrimmedInterfaceTypesWithoutDroppingOrderOrDuplicates)
{
  const auto request = protocol::interfaces::parse(
    nlohmann::json{
      {"interface_types", {" sensor_msgs/msg/BatteryState ", "std_msgs/msg/String", "sensor_msgs/msg/BatteryState "}},
      {"request_id", "ignored-by-parser"},
    }
      .dump());

  EXPECT_EQ(
    request.types,
    (std::vector<std::string>{
      "sensor_msgs/msg/BatteryState",
      "std_msgs/msg/String",
      "sensor_msgs/msg/BatteryState",
    }));
}

TEST(InterfacePayloadsTest, RejectsInvalidInterfaceTypeCollections)
{
  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse(R"({})"); }, "interface_types must be an array", "interface_types");
  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse(R"({"interface_types":"not_an_array"})"); },
    "interface_types must be an array",
    "interface_types");
  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse(R"({"interface_types":[]})"); },
    "interface_types must not be empty",
    "interface_types");
}

TEST(InterfacePayloadsTest, RejectsBlankInterfaceTypeEntryWithinOtherwiseValidArray)
{
  expectInvalidArgument(
    []() {
      (void)protocol::interfaces::parse(
        nlohmann::json{{"interface_types", {"sensor_msgs/msg/BatteryState", "   ", "std_msgs/msg/String"}}}.dump());
    },
    "interface_types entries must not be empty",
    "interface_types");
}

TEST(InterfacePayloadsTest, RejectsInvalidJsonAndNonObjectRequests)
{
  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse("{"); }, "Invalid JSON in interface show request", "payload");
  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse(R"(["sensor_msgs/msg/BatteryState"])"); },
    "Interface show request must be a JSON object",
    "payload");
}

TEST(InterfacePayloadsTest, RejectsNonStringInterfaceTypeEntries)
{
  expectInvalidArgument(
    []() {
      (void)protocol::interfaces::parse(
        nlohmann::json{{"interface_types", {"sensor_msgs/msg/BatteryState", 42}}}.dump());
    },
    "interface_types entries must be strings",
    "interface_types");
}

TEST(InterfacePayloadsTest, SerializesInterfacesByDirectFieldMappingWithoutReorderingOrDeduping)
{
  std::vector<InterfaceDefinition> definitions = {
    {"std_msgs/msg/Header", "ros2msg", "builtin_interfaces/Time stamp\nstring frame_id\n"},
    {"sensor_msgs/msg/BatteryState", "ros2msg", "float32 voltage\n"},
    {"std_msgs/msg/Header", "ros2msg", "builtin_interfaces/Time stamp\nstring frame_id\n"},
  };

  const auto body = protocol::interfaces::serialize(definitions);
  const auto expected = nlohmann::json{
    {"interfaces",
     {
       {
         {"interface_type", "std_msgs/msg/Header"},
         {"format", "ros2msg"},
         {"definition", "builtin_interfaces/Time stamp\nstring frame_id\n"},
       },
       {
         {"interface_type", "sensor_msgs/msg/BatteryState"},
         {"format", "ros2msg"},
         {"definition", "float32 voltage\n"},
       },
       {
         {"interface_type", "std_msgs/msg/Header"},
         {"format", "ros2msg"},
         {"definition", "builtin_interfaces/Time stamp\nstring frame_id\n"},
       },
     }}};

  EXPECT_EQ(nlohmann::json::parse(body), expected);
}

TEST(InterfacePayloadsTest, SerializesEmptyInterfaces)
{
  std::vector<InterfaceDefinition> definitions;

  const auto body = protocol::interfaces::serialize(definitions);
  const auto expected = nlohmann::json{{"interfaces", nlohmann::json::array()}};

  EXPECT_EQ(nlohmann::json::parse(body), expected);
}

}  // namespace

}  // namespace livekit_ros2_bridge
