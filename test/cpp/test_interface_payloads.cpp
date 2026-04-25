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
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/interfaces_json.hpp"
#include "protocol_test_support.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using test_support::expectInvalidArgument;

TEST(InterfacePayloadsTest, ParsesTrimmedInterfaceTypesWithoutDroppingOrderOrDuplicates)
{
  const std::string battery_state_type = rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();
  const std::string string_type = rosidl_generator_traits::name<std_msgs::msg::String>();

  const auto types = protocol::interfaces::parse(
    nlohmann::json{
      {"interface_types", {" " + battery_state_type + " ", string_type, battery_state_type + " "}},
      {"request_id", "ignored-by-parser"},
    }
      .dump());

  EXPECT_EQ(
    types,
    (std::vector<std::string>{
      battery_state_type,
      string_type,
      battery_state_type,
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
  const std::string battery_state_type = rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();
  const std::string string_type = rosidl_generator_traits::name<std_msgs::msg::String>();

  expectInvalidArgument(
    [&]() {
      (void)protocol::interfaces::parse(
        nlohmann::json{{"interface_types", {battery_state_type, "   ", string_type}}}.dump());
    },
    "interface_types entries must not be empty",
    "interface_types");
}

TEST(InterfacePayloadsTest, RejectsInvalidJsonAndNonObjectRequests)
{
  const std::string battery_state_type = rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();

  expectInvalidArgument(
    []() { (void)protocol::interfaces::parse("{"); }, "Invalid JSON in interface show request", "payload");
  expectInvalidArgument(
    [&]() { (void)protocol::interfaces::parse(nlohmann::json::array({battery_state_type}).dump()); },
    "Interface show request must be a JSON object",
    "payload");
}

TEST(InterfacePayloadsTest, RejectsNonStringInterfaceTypeEntries)
{
  const std::string battery_state_type = rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();

  expectInvalidArgument(
    [&]() { (void)protocol::interfaces::parse(nlohmann::json{{"interface_types", {battery_state_type, 42}}}.dump()); },
    "interface_types entries must be strings",
    "interface_types");
}

TEST(InterfacePayloadsTest, SerializesInterfacesByDirectFieldMappingWithoutReorderingOrDeduping)
{
  const std::string header_type = rosidl_generator_traits::name<std_msgs::msg::Header>();
  const std::string battery_state_type = rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();

  std::vector<InterfaceDefinition> definitions = {
    {header_type, "definition one\n"},
    {battery_state_type, "definition two\n"},
    {header_type, "definition one\n"},
  };

  const auto body = protocol::interfaces::serialize(definitions);
  const auto expected = nlohmann::json{
    {"interfaces",
     {
       {
         {"interface_type", header_type},
         {"format", "ros2msg"},
         {"definition", "definition one\n"},
       },
       {
         {"interface_type", battery_state_type},
         {"format", "ros2msg"},
         {"definition", "definition two\n"},
       },
       {
         {"interface_type", header_type},
         {"format", "ros2msg"},
         {"definition", "definition one\n"},
       },
     }}};

  EXPECT_EQ(nlohmann::json::parse(body), expected);
}

}  // namespace

}  // namespace livekit_ros2_bridge
