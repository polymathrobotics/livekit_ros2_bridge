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
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "wire/cdr.hpp"
#include "wire/services.hpp"

namespace livekit_ros2_bridge
{

namespace
{

template <typename Fn>
void expectInvalidArgument(Fn && fn, const char * expected_message, const char * expected_field = nullptr)
{
  try {
    fn();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(error.what(), std::string(expected_message));
    if (expected_field == nullptr) {
      return;
    }

    ASSERT_EQ(wire::services::invalidRequestField(error), std::optional<std::string_view>(expected_field));
  }
}

nlohmann::json makeRequestBody()
{
  return nlohmann::json{
    {"service", "/set_bool"},
    {"interface_type", "std_srvs/srv/SetBool"},
    {"request", wire::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
    {"timeout_ms", 500},
  };
}

TEST(ServiceCallPayloadsTest, ParsesValidRequestAndNormalizesFields)
{
  auto body = makeRequestBody();
  body["service"] = "  set_bool  ";
  body["interface_type"] = "  std_srvs/srv/SetBool  ";
  const auto request = wire::services::parse(body.dump());

  EXPECT_EQ(request.service, "/set_bool");
  EXPECT_EQ(request.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(request.request_payload, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
  EXPECT_EQ(request.timeout_ms, std::optional<int>(500));
}

TEST(ServiceCallPayloadsTest, ParsesOptionalInterfaceTypeAndPreservesTimeoutPresence)
{
  auto body = makeRequestBody();
  body.erase("interface_type");
  body.erase("timeout_ms");
  const auto request = wire::services::parse(body.dump());

  EXPECT_TRUE(request.interface_type.empty());
  EXPECT_EQ(request.timeout_ms, std::nullopt);

  body = makeRequestBody();
  body["interface_type"] = "   ";
  body["timeout_ms"] = 0;
  const auto zero_timeout_request = wire::services::parse(body.dump());

  EXPECT_TRUE(zero_timeout_request.interface_type.empty());
  EXPECT_EQ(zero_timeout_request.timeout_ms, std::optional<int>(0));

  body["timeout_ms"] = -1;
  const auto negative_timeout_request = wire::services::parse(body.dump());
  EXPECT_EQ(negative_timeout_request.timeout_ms, std::optional<int>(-1));
}

TEST(ServiceCallPayloadsTest, RejectsMalformedPayloadsWithFieldContext)
{
  expectInvalidArgument([]() { (void)wire::services::parse("{"); }, "Invalid JSON in service call request", "payload");
  expectInvalidArgument(
    []() { (void)wire::services::parse(R"([1,2,3])"); }, "Service call request must be a JSON object", "payload");
}

TEST(ServiceCallPayloadsTest, RejectsInvalidRequestFieldsWithFieldContext)
{
  auto body = makeRequestBody();
  body.erase("service");
  expectInvalidArgument([&body]() { (void)wire::services::parse(body.dump()); }, "service is required", "service");

  body = makeRequestBody();
  body["service"] = "   ";
  expectInvalidArgument([&body]() { (void)wire::services::parse(body.dump()); }, "service is required", "service");

  body = makeRequestBody();
  body["interface_type"] = 123;
  expectInvalidArgument(
    [&body]() { (void)wire::services::parse(body.dump()); }, "interface_type must be a string", "interface_type");

  body = makeRequestBody();
  body["request"] = wire::cdr::serialize(std::vector<std::uint8_t>{});
  expectInvalidArgument(
    [&body]() { (void)wire::services::parse(body.dump()); }, "request.payload_base64 must not be empty", "request");

  body = makeRequestBody();
  body["timeout_ms"] = nullptr;
  expectInvalidArgument(
    [&body]() { (void)wire::services::parse(body.dump()); }, "timeout_ms must be an integer", "timeout_ms");
}

TEST(ServiceCallPayloadsTest, SerializesResponse)
{
  const auto payload =
    wire::services::serialize("/set_bool", "std_srvs/srv/SetBool", std::vector<std::uint8_t>{0x01, 0x02});

  EXPECT_EQ(
    nlohmann::json::parse(payload),
    nlohmann::json({
      {"service", "/set_bool"},
      {"interface_type", "std_srvs/srv/SetBool"},
      {"response", wire::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02})},
    }));
}

}  // namespace

}  // namespace livekit_ros2_bridge
