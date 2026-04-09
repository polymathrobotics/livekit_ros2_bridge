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

#include <array>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "payloads/service_call_payloads.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(ServiceCallPayloadsTest, ParsesValidRequestAndNormalizesFields)
{
  const auto request = parseServiceCallRequest(
    nlohmann::json{
      {"service", "  set_bool  "},
      {"interface_type", "  std_srvs/srv/SetBool  "},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
      {"timeout_ms", 500},
    }.dump());

  EXPECT_EQ(request.service, "/set_bool");
  EXPECT_EQ(request.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(request.request, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
  EXPECT_EQ(request.timeout_ms, 500);
}

TEST(ServiceCallPayloadsTest, ParsesRequestWithoutOptionalFields)
{
  const auto request = parseServiceCallRequest(
    nlohmann::json{
      {"service", "/trigger"},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    }.dump());

  EXPECT_EQ(request.service, "/trigger");
  EXPECT_TRUE(request.interface_type.empty());
  EXPECT_EQ(request.request, (std::vector<std::uint8_t>{0x01}));
  EXPECT_EQ(request.timeout_ms, 0);
}

TEST(ServiceCallPayloadsTest, RejectsEmptyRequestPayload)
{
  EXPECT_THROW(
    parseServiceCallRequest(
      nlohmann::json{
        {"service", "/foo"},
        {"request", {{"content_type", "application/x-ros-cdr"}, {"payload_base64", ""}}},
      }.dump()),
    std::invalid_argument);
}

TEST(ServiceCallPayloadsTest, RejectsInvalidJson)
{
  EXPECT_THROW(parseServiceCallRequest("{"), std::invalid_argument);
}

TEST(ServiceCallPayloadsTest, RejectsMissingOrBlankService)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json{{"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})}},
    nlohmann::json{
      {"service", ""},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },
    nlohmann::json{
      {"service", "   "},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    }};

  for (const auto & payload : payloads) {
    EXPECT_THROW(parseServiceCallRequest(payload.dump()), std::invalid_argument);
  }
}

TEST(ServiceCallPayloadsTest, RejectsMissingOrMalformedRequest)
{
  const std::array<nlohmann::json, 2> payloads{
    nlohmann::json{{"service", "/foo"}},
    nlohmann::json{{"service", "/foo"}, {"request", nlohmann::json::array({1, 2})}}};

  for (const auto & payload : payloads) {
    EXPECT_THROW(parseServiceCallRequest(payload.dump()), std::invalid_argument);
  }
}

TEST(ServiceCallPayloadsTest, RejectsMistypedOptionalFields)
{
  const auto valid_request = serializeCdrPayload(std::vector<std::uint8_t>{0x01});
  const std::array<nlohmann::json, 4> payloads{
    nlohmann::json{
      {"service", "/foo"},
      {"interface_type", 123},
      {"request", valid_request},
    },
    nlohmann::json{
      {"service", "/foo"},
      {"interface_type", nullptr},
      {"request", valid_request},
    },
    nlohmann::json{
      {"service", "/foo"},
      {"timeout_ms", "500"},
      {"request", valid_request},
    },
    nlohmann::json{
      {"service", "/foo"},
      {"timeout_ms", 5.5},
      {"request", valid_request},
    }};

  for (const auto & payload : payloads) {
    EXPECT_THROW(parseServiceCallRequest(payload.dump()), std::invalid_argument);
  }
}

TEST(ServiceCallPayloadsTest, IgnoresEmptyTypeString)
{
  const auto request = parseServiceCallRequest(
    nlohmann::json{
      {"service", "/foo"},
      {"interface_type", ""},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    }.dump());

  EXPECT_TRUE(request.interface_type.empty());
}

TEST(ServiceCallPayloadsTest, SerializesResponse)
{
  const auto serialized =
    serializeServiceCallResponse("/set_bool", "std_srvs/srv/SetBool", std::vector<std::uint8_t>{0x01, 0x02}, 42);
  const auto body = nlohmann::json::parse(serialized);

  EXPECT_TRUE(body["ok"].get<bool>());
  EXPECT_EQ(body["service"]["name"].get<std::string>(), "/set_bool");
  EXPECT_EQ(body["service"]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
  EXPECT_EQ(parseCdrPayload(body, "response"), (std::vector<std::uint8_t>{0x01, 0x02}));
  EXPECT_EQ(body["elapsed_ms"].get<int>(), 42);
}

}  // namespace

}  // namespace livekit_ros2_bridge
