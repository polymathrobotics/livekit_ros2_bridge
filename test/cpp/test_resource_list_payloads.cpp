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
#include <stdexcept>
#include <string_view>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "wire/resources.hpp"

namespace livekit_ros2_bridge
{

namespace
{
void expectInvalidRequestField(
  const std::string & payload, std::string_view expected_field, const char * expected_message)
{
  try {
    (void)wire::resources::parse(payload);
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(std::string(error.what()), std::string(expected_message));
    EXPECT_EQ(wire::resources::invalidRequestField(error), std::optional<std::string_view>(expected_field));
  }
}

TEST(ResourceListPayloadsTest, IgnoresUnknownFieldsAndTreatsBlankOptionalsAsAbsent)
{
  const auto request = wire::resources::parse(R"({
    "extra":"ignored",
    "query":"   ",
    "limit":null
  })");

  EXPECT_EQ(request.query, std::nullopt);
  EXPECT_EQ(request.limit, std::nullopt);

  const auto null_request = wire::resources::parse(R"({"query":null})");

  EXPECT_EQ(null_request.query, std::nullopt);
  EXPECT_EQ(null_request.limit, std::nullopt);
}

TEST(ResourceListPayloadsTest, ParsesTrimmedQueryAndLimit)
{
  const auto request = wire::resources::parse(R"({"query":" /cortex/modify ","limit":25})");

  EXPECT_EQ(request.query, std::optional<std::string>("/cortex/modify"));
  EXPECT_EQ(request.limit, std::optional<std::size_t>(25u));
}

TEST(ResourceListPayloadsTest, RejectsInvalidQueryAndLimitFields)
{
  EXPECT_THROW(wire::resources::parse(R"({"query":123})"), std::invalid_argument);
  EXPECT_THROW(wire::resources::parse(R"({"limit":-1})"), std::invalid_argument);
  EXPECT_THROW(wire::resources::parse(R"({"limit":0})"), std::invalid_argument);
  EXPECT_THROW(wire::resources::parse(R"({"limit":1.5})"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, RejectsMalformedJsonAndNonObjectPayloads)
{
  EXPECT_THROW(wire::resources::parse("{"), std::invalid_argument);
  EXPECT_THROW(wire::resources::parse(R"([])"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, ReportsRejectedRequestFieldForValidationFailures)
{
  expectInvalidRequestField("{", "payload", "Invalid JSON in list request");
  expectInvalidRequestField(R"([])", "payload", "List request must be a JSON object");
  expectInvalidRequestField(R"({"query":123})", "query", "query must be a string");
  expectInvalidRequestField(R"({"limit":0})", "limit", "limit must be a positive integer");

  const std::invalid_argument unrelated_error("other_validation_error");
  EXPECT_EQ(wire::resources::invalidRequestField(unrelated_error), std::nullopt);
}

TEST(ResourceListPayloadsTest, SerializesServices)
{
  const auto body = nlohmann::json::parse(
    wire::resources::serializeServices({
      {"/set_bool", "std_srvs/srv/SetBool"},
      {"/trigger", "std_srvs/srv/Trigger"},
    }));

  EXPECT_EQ(
    body,
    nlohmann::json({
      {"services",
       nlohmann::json::array({
         {{"name", "/set_bool"}, {"interface_type", "std_srvs/srv/SetBool"}},
         {{"name", "/trigger"}, {"interface_type", "std_srvs/srv/Trigger"}},
       })},
    }));

  EXPECT_EQ(
    nlohmann::json::parse(wire::resources::serializeServices({})),
    nlohmann::json({{"services", nlohmann::json::array()}}));
}

TEST(ResourceListPayloadsTest, SerializesTopics)
{
  EXPECT_EQ(
    nlohmann::json::parse(
      wire::resources::serializeTopics({
        {"/camera/image_raw", "sensor_msgs/msg/Image"},
      })),
    nlohmann::json({
      {"topics",
       nlohmann::json::array({
         {{"name", "/camera/image_raw"}, {"interface_type", "sensor_msgs/msg/Image"}},
       })},
    }));

  EXPECT_EQ(
    nlohmann::json::parse(wire::resources::serializeTopics({})), nlohmann::json({{"topics", nlohmann::json::array()}}));
}

}  // namespace

}  // namespace livekit_ros2_bridge
