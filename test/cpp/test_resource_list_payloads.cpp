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

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/resource_list_payloads.hpp"

namespace livekit_ros2_bridge
{

namespace
{
TEST(ResourceListPayloadsTest, IgnoresUnknownFieldsAndTreatsBlankOptionalsAsAbsent)
{
  const auto request = parseResourceListRequest(R"({
    "extra":"ignored",
    "query":"   ",
    "limit":null
  })");

  EXPECT_EQ(request.query, std::nullopt);
  EXPECT_EQ(request.limit, std::nullopt);

  const auto null_query_request = parseResourceListRequest(R"({"query":null})");

  EXPECT_EQ(null_query_request.query, std::nullopt);
  EXPECT_EQ(null_query_request.limit, std::nullopt);
}

TEST(ResourceListPayloadsTest, ParsesTrimmedQueryAndLimit)
{
  const auto request = parseResourceListRequest(R"({"query":" /cortex/modify ","limit":25})");

  EXPECT_EQ(request.query, std::optional<std::string>("/cortex/modify"));
  EXPECT_EQ(request.limit, std::optional<std::size_t>(25u));
}

TEST(ResourceListPayloadsTest, RejectsInvalidQueryAndLimitFields)
{
  EXPECT_THROW(parseResourceListRequest(R"({"query":123})"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"({"limit":-1})"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"({"limit":0})"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"({"limit":1.5})"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, RejectsMalformedJsonAndNonObjectPayloads)
{
  EXPECT_THROW(parseResourceListRequest("{"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"([])"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, SerializesServiceList)
{
  const auto body = nlohmann::json::parse(serializeServiceListResponse({
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
}

TEST(ResourceListPayloadsTest, SerializesEmptyTopicList)
{
  EXPECT_EQ(
    nlohmann::json::parse(serializeTopicListResponse({})), nlohmann::json({{"topics", nlohmann::json::array()}}));
}

}  // namespace

}  // namespace livekit_ros2_bridge
