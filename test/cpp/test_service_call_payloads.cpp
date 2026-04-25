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

#include <chrono>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/services_json.hpp"
#include "protocol_test_support.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using test_support::expectInvalidArgument;

std::vector<std::uint8_t> serializedMessageBytes(const rclcpp::SerializedMessage & message)
{
  const auto & raw = message.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(raw.buffer, raw.buffer + message.size());
}

nlohmann::json makeRequestBody()
{
  return nlohmann::json{
    {"service", "/set_bool"},
    {"interface_type", "std_srvs/srv/SetBool"},
    {"request", protocol::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
    {"timeout_ms", 500},
  };
}

TEST(ServiceCallPayloadsTest, ParsesValidRequestAndExpandsFields)
{
  auto body = makeRequestBody();
  body["service"] = "  set_bool  ";
  body["interface_type"] = "  std_srvs/srv/SetBool  ";
  const auto request = protocol::services::parse(body.dump());

  EXPECT_EQ(request.name, "/set_bool");
  EXPECT_EQ(request.interface_type, "std_srvs/srv/SetBool");
  EXPECT_EQ(serializedMessageBytes(request.payload), (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
  EXPECT_EQ(request.timeout, std::chrono::milliseconds(500));
}

TEST(ServiceCallPayloadsTest, ParsesOptionalInterfaceTypeAndPreservesTimeoutPresence)
{
  auto body = makeRequestBody();
  body.erase("interface_type");
  body.erase("timeout_ms");
  const auto request = protocol::services::parse(body.dump());

  EXPECT_TRUE(request.interface_type.empty());
  EXPECT_EQ(request.timeout, std::nullopt);

  body = makeRequestBody();
  body["interface_type"] = "   ";
  body["timeout_ms"] = 0;
  const auto zero_timeout = protocol::services::parse(body.dump());

  EXPECT_TRUE(zero_timeout.interface_type.empty());
  EXPECT_EQ(zero_timeout.timeout, std::chrono::milliseconds(0));

  body["timeout_ms"] = -1;
  const auto negative_timeout = protocol::services::parse(body.dump());
  EXPECT_EQ(negative_timeout.timeout, std::chrono::milliseconds(-1));
}

TEST(ServiceCallPayloadsTest, RejectsMalformedPayloadsWithFieldContext)
{
  expectInvalidArgument(
    []() { (void)protocol::services::parse("{"); }, "Invalid JSON in service call request", "payload");
  expectInvalidArgument(
    []() { (void)protocol::services::parse(R"([1,2,3])"); }, "Service call request must be a JSON object", "payload");
}

TEST(ServiceCallPayloadsTest, RejectsInvalidRequestFieldsWithFieldContext)
{
  auto body = makeRequestBody();
  body.erase("service");
  expectInvalidArgument([&body]() { (void)protocol::services::parse(body.dump()); }, "service is required", "service");

  body = makeRequestBody();
  body["service"] = "   ";
  expectInvalidArgument([&body]() { (void)protocol::services::parse(body.dump()); }, "service is required", "service");

  body = makeRequestBody();
  body["service"] = "/bad//service";
  try {
    (void)protocol::services::parse(body.dump());
    ADD_FAILURE() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    const auto * validation = dynamic_cast<const protocol::ValidationError *>(&error);
    ASSERT_NE(validation, nullptr);
    EXPECT_EQ(validation->field(), "service");
  }

  body = makeRequestBody();
  body["interface_type"] = 123;
  expectInvalidArgument(
    [&body]() { (void)protocol::services::parse(body.dump()); }, "interface_type must be a string", "interface_type");

  body = makeRequestBody();
  body["request"] = protocol::cdr::serialize(std::vector<std::uint8_t>{});
  expectInvalidArgument(
    [&body]() { (void)protocol::services::parse(body.dump()); }, "request.payload_base64 must not be empty", "request");

  body = makeRequestBody();
  body["timeout_ms"] = nullptr;
  expectInvalidArgument(
    [&body]() { (void)protocol::services::parse(body.dump()); }, "timeout_ms must be an integer", "timeout_ms");
}

TEST(ServiceCallPayloadsTest, SerializesResponse)
{
  const auto body = protocol::services::serialize(
    ServiceCallResponse{"/set_bool", "std_srvs/srv/SetBool", std::vector<std::uint8_t>{0x01, 0x02}});

  EXPECT_EQ(
    nlohmann::json::parse(body),
    nlohmann::json({
      {"service", "/set_bool"},
      {"interface_type", "std_srvs/srv/SetBool"},
      {"response", protocol::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02})},
    }));
}

}  // namespace

}  // namespace livekit_ros2_bridge
