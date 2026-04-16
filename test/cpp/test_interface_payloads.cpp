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
#include "wire/interfaces.hpp"

namespace livekit_ros2_bridge
{

namespace
{

template <typename Fn>
void expectInvalidArgument(Fn && fn, const char * expected_message, const char * expected_field = nullptr)
{
  try {
    fn();
    ADD_FAILURE() << "Expected std::invalid_argument";
    return;
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(error.what(), std::string(expected_message));
    if (expected_field == nullptr) {
      return;
    }

    const auto invalid_field = wire::interfaces::invalidRequestField(error);
    ASSERT_TRUE(invalid_field.has_value());
    EXPECT_EQ(*invalid_field, expected_field);
  }
}

TEST(InterfacePayloadsTest, ParsesTrimmedInterfaceTypesWithoutDroppingOrderOrDuplicates)
{
  const auto interface_types = wire::interfaces::parse(
    nlohmann::json{
      {"interface_types", {" sensor_msgs/msg/BatteryState ", "std_msgs/msg/String", "sensor_msgs/msg/BatteryState "}},
      {"request_id", "ignored-by-parser"},
    }
      .dump());

  EXPECT_EQ(
    interface_types,
    (std::vector<std::string>{
      "sensor_msgs/msg/BatteryState",
      "std_msgs/msg/String",
      "sensor_msgs/msg/BatteryState",
    }));
}

TEST(InterfacePayloadsTest, RejectsInvalidInterfaceTypeCollections)
{
  expectInvalidArgument([]() { (void)wire::interfaces::parse(R"({})"); }, "interface_types must be an array");
  expectInvalidArgument(
    []() { (void)wire::interfaces::parse(R"({"interface_types":"not_an_array"})"); },
    "interface_types must be an array");
  expectInvalidArgument(
    []() { (void)wire::interfaces::parse(R"({"interface_types":[]})"); }, "interface_types must not be empty");
}

TEST(InterfacePayloadsTest, RejectsBlankInterfaceTypeEntryWithinOtherwiseValidArray)
{
  expectInvalidArgument(
    []() {
      (void)wire::interfaces::parse(
        nlohmann::json{{"interface_types", {"sensor_msgs/msg/BatteryState", "   ", "std_msgs/msg/String"}}}.dump());
    },
    "interface_types entries must not be empty");
}

TEST(InterfacePayloadsTest, RejectsInvalidJsonAndNonObjectRequests)
{
  expectInvalidArgument(
    []() { (void)wire::interfaces::parse("{"); }, "Invalid JSON in interface show request", "payload");
  expectInvalidArgument(
    []() { (void)wire::interfaces::parse(R"(["sensor_msgs/msg/BatteryState"])"); },
    "Interface show request must be a JSON object");
}

TEST(InterfacePayloadsTest, RejectsNonStringInterfaceTypeEntries)
{
  expectInvalidArgument(
    []() {
      (void)wire::interfaces::parse(nlohmann::json{{"interface_types", {"sensor_msgs/msg/BatteryState", 42}}}.dump());
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

  const auto serialized = wire::interfaces::serialize(definitions);
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

  EXPECT_EQ(nlohmann::json::parse(serialized), expected);
}

TEST(InterfacePayloadsTest, SerializesEmptyInterfaces)
{
  std::vector<InterfaceDefinition> definitions;

  const auto serialized = wire::interfaces::serialize(definitions);
  const auto expected = nlohmann::json{{"interfaces", nlohmann::json::array()}};

  EXPECT_EQ(nlohmann::json::parse(serialized), expected);
}

}  // namespace

}  // namespace livekit_ros2_bridge
