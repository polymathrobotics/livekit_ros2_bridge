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

#include <optional>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/resources_json.hpp"
#include "protocol_test_support.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using test_support::expectInvalidArgument;

std::string imageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::Image>();
}

std::string setCameraInfoInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::srv::SetCameraInfo>();
}

TEST(ResourceListPayloadsTest, IgnoresUnknownFieldsAndTreatsBlankOptionalsAsAbsent)
{
  const auto request = protocol::resources::parseRequest(R"({
    "extra":"ignored",
    "query":"   ",
    "limit":null
  })");

  EXPECT_EQ(request.query, std::nullopt);
  EXPECT_EQ(request.limit, std::nullopt);

  const auto null_request = protocol::resources::parseRequest(R"({"query":null})");

  EXPECT_EQ(null_request.query, std::nullopt);
  EXPECT_EQ(null_request.limit, std::nullopt);
}

TEST(ResourceListPayloadsTest, ParsesTrimmedQueryAndLimit)
{
  const auto request = protocol::resources::parseRequest(R"({"query":" /cortex/modify ","limit":25})");

  EXPECT_EQ(request.query, std::optional<std::string>("/cortex/modify"));
  EXPECT_EQ(request.limit, std::optional<std::size_t>(25u));
}

TEST(ResourceListPayloadsTest, RejectsInvalidRequestsWithFieldContext)
{
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest("{"); }, "Invalid JSON in list request", "payload");
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest(R"([])"); }, "List request must be a JSON object", "payload");
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest(R"({"query":123})"); }, "query must be a string", "query");
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest(R"({"limit":-1})"); }, "limit must be a positive integer", "limit");
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest(R"({"limit":0})"); }, "limit must be a positive integer", "limit");
  expectInvalidArgument(
    []() { (void)protocol::resources::parseRequest(R"({"limit":1.5})"); }, "limit must be a positive integer", "limit");
}

TEST(ResourceListPayloadsTest, SerializesServices)
{
  const auto interface_type = setCameraInfoInterfaceType();

  const auto body = nlohmann::json::parse(
    protocol::resources::serializeServices({
      {"/backup_camera_info", {interface_type}},
      {"/set_camera_info", {interface_type}},
    }));

  EXPECT_EQ(
    body,
    nlohmann::json({
      {"services",
       nlohmann::json::array({
         {{"service", "/backup_camera_info"}, {"interface_type", interface_type}},
         {{"service", "/set_camera_info"}, {"interface_type", interface_type}},
       })},
    }));

  EXPECT_EQ(
    nlohmann::json::parse(protocol::resources::serializeServices({})),
    nlohmann::json({{"services", nlohmann::json::array()}}));
}

TEST(ResourceListPayloadsTest, SerializesTopics)
{
  const auto interface_type = imageInterfaceType();

  EXPECT_EQ(
    nlohmann::json::parse(
      protocol::resources::serializeTopics({
        {"/camera/image_raw", {interface_type}},
      })),
    nlohmann::json({
      {"topics",
       nlohmann::json::array({
         {{"topic", "/camera/image_raw"}, {"interface_type", interface_type}},
       })},
    }));

  EXPECT_EQ(
    nlohmann::json::parse(protocol::resources::serializeTopics({})),
    nlohmann::json({{"topics", nlohmann::json::array()}}));
}

}  // namespace

}  // namespace livekit_ros2_bridge
