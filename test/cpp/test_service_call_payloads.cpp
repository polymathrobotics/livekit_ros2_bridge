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
#include <string_view>
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
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(error.what(), std::string(expected_message));
  }
}

void expectParseError(const nlohmann::json & body, const char * expected_message)
{
  expectInvalidArgumentMessage([&body]() { (void)service_call_payloads::parse(body.dump()); }, expected_message);
}

template <typename Fn>
void expectInvalidRequestField(Fn && fn, const char * expected_field)
{
  try {
    fn();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    ASSERT_EQ(service_call_payloads::invalidRequestField(error), std::string_view(expected_field));
  }
}

nlohmann::json makeRequestBody()
{
  return nlohmann::json{
    {"service", "/set_bool"},
    {"interface_type", "std_srvs/srv/SetBool"},
    {"request", cdr_payload::serialize(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
    {"timeout_ms", 500},
  };
}

TEST(ServiceCallPayloadsTest, ParsesValidRequestAndNormalizesFields)
{
  auto body = makeRequestBody();
  body["service"] = "  set_bool  ";
  body["interface_type"] = "  std_srvs/srv/SetBool  ";
  const auto request = service_call_payloads::parse(body.dump());

  EXPECT_EQ(request.service, "/set_bool");
  EXPECT_EQ(request.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(request.request_payload, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
  ASSERT_TRUE(request.timeout_ms.has_value());
  EXPECT_EQ(*request.timeout_ms, 500);
}

TEST(ServiceCallPayloadsTest, ParsesOptionalInterfaceTypeAndPreservesTimeoutPresence)
{
  auto body = makeRequestBody();
  body.erase("interface_type");
  body.erase("timeout_ms");
  const auto request = service_call_payloads::parse(body.dump());

  EXPECT_TRUE(request.interface_type.empty());
  EXPECT_EQ(request.timeout_ms, std::nullopt);

  body = makeRequestBody();
  body["interface_type"] = "   ";
  body["timeout_ms"] = 0;
  const auto zero_timeout_request = service_call_payloads::parse(body.dump());

  EXPECT_TRUE(zero_timeout_request.interface_type.empty());
  EXPECT_EQ(zero_timeout_request.timeout_ms, std::optional<int>(0));

  body["timeout_ms"] = -1;
  const auto negative_timeout_request = service_call_payloads::parse(body.dump());
  EXPECT_EQ(negative_timeout_request.timeout_ms, std::optional<int>(-1));
}

TEST(ServiceCallPayloadsTest, RejectsInvalidJson)
{
  expectInvalidArgumentMessage(
    []() { (void)service_call_payloads::parse("{"); }, "Invalid JSON in service call request");
}

TEST(ServiceCallPayloadsTest, RejectsNonObjectRoot)
{
  expectInvalidArgumentMessage(
    []() { (void)service_call_payloads::parse(R"([1,2,3])"); }, "Service call request must be a JSON object");
}

TEST(ServiceCallPayloadsTest, ReportsInvalidRequestFieldForRejectedPayloads)
{
  expectInvalidRequestField([]() { (void)service_call_payloads::parse("{"); }, "payload");

  auto body = makeRequestBody();
  body.erase("service");
  expectInvalidRequestField([&body]() { (void)service_call_payloads::parse(body.dump()); }, "service");

  body = makeRequestBody();
  body["interface_type"] = 123;
  expectInvalidRequestField([&body]() { (void)service_call_payloads::parse(body.dump()); }, "interface_type");

  body = makeRequestBody();
  body["request"] = cdr_payload::serialize(std::vector<std::uint8_t>{});
  expectInvalidRequestField([&body]() { (void)service_call_payloads::parse(body.dump()); }, "request");

  body = makeRequestBody();
  body["timeout_ms"] = "500";
  expectInvalidRequestField([&body]() { (void)service_call_payloads::parse(body.dump()); }, "timeout_ms");
}

TEST(ServiceCallPayloadsTest, RejectsMissingServiceAndEmptyRequestPayload)
{
  auto body = makeRequestBody();
  body.erase("service");
  expectParseError(body, "service is required");

  body = makeRequestBody();
  body["service"] = "   ";
  expectParseError(body, "service is required");

  body = makeRequestBody();
  body["request"] = cdr_payload::serialize(std::vector<std::uint8_t>{});
  expectParseError(body, "request.payload_base64 must not be empty");
}

TEST(ServiceCallPayloadsTest, RejectsMistypedOptionalFields)
{
  expectParseError(
    nlohmann::json{
      {"service", "/foo"},
      {"timeout_ms", "500"},
      {"request", cdr_payload::serialize(std::vector<std::uint8_t>{0x01})},
    },
    "timeout_ms must be an integer");

  expectParseError(
    nlohmann::json{
      {"service", "/foo"},
      {"interface_type", 123},
      {"request", cdr_payload::serialize(std::vector<std::uint8_t>{0x01})},
    },
    "interface_type must be a string");

  expectParseError(
    nlohmann::json{
      {"service", "/foo"},
      {"timeout_ms", nullptr},
      {"request", cdr_payload::serialize(std::vector<std::uint8_t>{0x01})},
    },
    "timeout_ms must be an integer");
}

TEST(ServiceCallPayloadsTest, SerializesResponse)
{
  const auto payload =
    service_call_payloads::serialize("/set_bool", "std_srvs/srv/SetBool", std::vector<std::uint8_t>{0x01, 0x02}, 42);
  const auto body = nlohmann::json::parse(payload);

  EXPECT_TRUE(body["ok"].get<bool>());
  EXPECT_EQ(body["service"]["name"].get<std::string>(), "/set_bool");
  EXPECT_EQ(body["service"]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
  EXPECT_EQ(cdr_payload::parse(body, "response"), (std::vector<std::uint8_t>{0x01, 0x02}));
  EXPECT_EQ(body["elapsed_ms"].get<int>(), 42);
}

}  // namespace

}  // namespace livekit_ros2_bridge
