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
#include "payloads/cdr_payload.hpp"
#include "payloads/service_call_payloads.hpp"

namespace livekit_ros2_bridge
{

namespace
{

template <typename Fn>
void expectInvalidArgumentMessage(Fn && fn, const char * expected_message)
{
  try {
    fn();
    ADD_FAILURE() << "Expected std::invalid_argument";
    return;
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(error.what(), std::string(expected_message));
  }
}

void expectRequestError(const nlohmann::json & body, const char * expected_message)
{
  expectInvalidArgumentMessage([&body]() { (void)parseServiceCallRequest(body.dump()); }, expected_message);
}

nlohmann::json makeValidRequestBody()
{
  return nlohmann::json{
    {"service", "/set_bool"},
    {"interface_type", "std_srvs/srv/SetBool"},
    {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
    {"timeout_ms", 500},
  };
}

TEST(ServiceCallPayloadsTest, ParsesValidRequestAndNormalizesFields)
{
  auto body = makeValidRequestBody();
  body["service"] = "  set_bool  ";
  body["interface_type"] = "  std_srvs/srv/SetBool  ";
  const auto request = parseServiceCallRequest(body.dump());

  EXPECT_EQ(request.service, "/set_bool");
  EXPECT_EQ(request.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(request.request_payload, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
  EXPECT_EQ(request.timeout_ms, 500);
}

TEST(ServiceCallPayloadsTest, ParsesOptionalInterfaceTypeAndTimeoutVariants)
{
  auto body = makeValidRequestBody();
  body.erase("interface_type");
  body.erase("timeout_ms");
  const auto defaulted_request = parseServiceCallRequest(body.dump());

  EXPECT_TRUE(defaulted_request.interface_type.empty());
  EXPECT_EQ(defaulted_request.timeout_ms, 0);

  body = makeValidRequestBody();
  body["interface_type"] = "   ";
  body["timeout_ms"] = -1;
  const auto explicit_request = parseServiceCallRequest(body.dump());

  EXPECT_TRUE(explicit_request.interface_type.empty());
  EXPECT_EQ(explicit_request.timeout_ms, -1);
}

TEST(ServiceCallPayloadsTest, RejectsInvalidJson)
{
  expectInvalidArgumentMessage([]() { (void)parseServiceCallRequest("{"); }, "Invalid JSON in service call request");
}

TEST(ServiceCallPayloadsTest, RejectsNonObjectRoot)
{
  expectInvalidArgumentMessage(
    []() { (void)parseServiceCallRequest(R"([1,2,3])"); }, "Service call request must be a JSON object");
}

TEST(ServiceCallPayloadsTest, RejectsMissingServiceAndEmptyRequestPayload)
{
  auto body = makeValidRequestBody();
  body.erase("service");
  expectRequestError(body, "service is required");

  body = makeValidRequestBody();
  body["service"] = "   ";
  expectRequestError(body, "service is required");

  // TODO: Keep request-envelope structure/content-type/base64 cases in test_payload_helpers.cpp;
  // this file should only cover the service-call-specific non-empty request rule.
  body = makeValidRequestBody();
  body["request"] = serializeCdrPayload(std::vector<std::uint8_t>{});
  expectRequestError(body, "request.payload_base64 must not be empty");
}

TEST(ServiceCallPayloadsTest, RejectsMistypedOptionalFields)
{
  expectRequestError(
    nlohmann::json{
      {"service", "/foo"},
      {"timeout_ms", "500"},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },
    "timeout_ms must be an integer");

  expectRequestError(
    nlohmann::json{
      {"service", "/foo"},
      {"interface_type", 123},
      {"request", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },
    "interface_type must be a string");
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
