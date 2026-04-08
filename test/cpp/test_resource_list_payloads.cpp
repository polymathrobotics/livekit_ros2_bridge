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

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/resource_list_payloads.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using SerializeListResponseFn = std::string (*)(const std::vector<ResourceListEntry> & entries);

void expectSerializedListResponse(
  SerializeListResponseFn serialize, const char * key, const std::vector<ResourceListEntry> & entries)
{
  nlohmann::json expected_entries = nlohmann::json::array();
  for (const auto & entry : entries) {
    expected_entries.push_back({{"name", entry.name}, {"interface_type", entry.interface_type}});
  }

  const nlohmann::json expected_body = {{key, expected_entries}};
  EXPECT_EQ(nlohmann::json::parse(serialize(entries)), expected_body);
}

TEST(ResourceListPayloadsTest, IgnoresUnknownFieldsAndNullOptionals)
{
  const auto request = parseResourceListRequest(R"({"query":null,"limit":null,"extra":"ignored"})");

  EXPECT_FALSE(request.query.has_value());
  EXPECT_FALSE(request.limit.has_value());
}

TEST(ResourceListPayloadsTest, ParsesQueryAndLimit)
{
  const auto request = parseResourceListRequest(R"({"query":" /cortex/modify ","limit":25})");

  ASSERT_TRUE(request.query.has_value());
  EXPECT_EQ(*request.query, "/cortex/modify");
  ASSERT_TRUE(request.limit.has_value());
  EXPECT_EQ(*request.limit, 25u);
}

TEST(ResourceListPayloadsTest, RejectsInvalidJson)
{
  EXPECT_THROW(parseResourceListRequest("{"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, RejectsNonObject)
{
  EXPECT_THROW(parseResourceListRequest("[1,2]"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, RejectsInvalidQueryType)
{
  EXPECT_THROW(parseResourceListRequest(R"({"query":123})"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, RejectsNonPositiveOrNonIntegralLimit)
{
  EXPECT_THROW(parseResourceListRequest(R"({"limit":0})"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"({"limit":-1})"), std::invalid_argument);
  EXPECT_THROW(parseResourceListRequest(R"({"limit":1.5})"), std::invalid_argument);
}

TEST(ResourceListPayloadsTest, SerializesServiceList)
{
  expectSerializedListResponse(
    serializeServiceListResponse,
    "services",
    {
      {"/set_bool", "std_srvs/srv/SetBool"},
      {"/trigger", "std_srvs/srv/Trigger"},
    });
}

TEST(ResourceListPayloadsTest, SerializesEmptyTopicList)
{
  expectSerializedListResponse(serializeTopicListResponse, "topics", {});
}

}  // namespace

}  // namespace livekit_ros2_bridge
