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

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol_test_support.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using test_support::expectInvalidArgument;

nlohmann::json makeMessageBody()
{
  return nlohmann::json{
    {"message",
     {
       {"content_type", "application/x-ros-cdr"},
       {"payload_base64", "AQID"},
     }},
  };
}

std::vector<std::uint8_t> payloadBytes(const rclcpp::SerializedMessage & message)
{
  if (message.size() == 0U) {
    return {};
  }

  const auto & rcl_message = message.get_rcl_serialized_message();
  return {rcl_message.buffer, rcl_message.buffer + message.size()};
}

std::vector<std::uint8_t> parsePayload(const nlohmann::json & body, protocol::cdr::Field field)
{
  return payloadBytes(protocol::cdr::parse(body, field));
}

TEST(PayloadHelpersTest, ParseObjectRejectsInvalidJsonAndNonObjectRoot)
{
  expectInvalidArgument(
    []() { (void)protocol::detail::parseObject("{", "payload JSON is invalid", "payload must be a JSON object"); },
    "payload JSON is invalid");
  expectInvalidArgument(
    []() {
      (void)protocol::detail::parseObject("[1,2,3]", "payload JSON is invalid", "payload must be a JSON object");
    },
    "payload must be a JSON object");
}

TEST(PayloadHelpersTest, ParseObjectAcceptsByteVectorPayloads)
{
  const std::string payload = R"({"name":" /camera "})";
  const auto body = protocol::detail::parseObject(
    std::vector<std::uint8_t>(payload.begin(), payload.end()),
    "payload JSON is invalid",
    "payload must be a JSON object");

  EXPECT_EQ(body, nlohmann::json({{"name", " /camera "}}));
}

TEST(PayloadHelpersTest, RequiredStringTrimsAndRejectsInvalidValues)
{
  const nlohmann::json body = {
    {"trimmed", "  /camera/image  "},
    {"blank", "   "},
    {"wrong_type", 42},
  };

  EXPECT_EQ(protocol::detail::requiredString(body, "trimmed", "field is required"), "/camera/image");

  expectInvalidArgument(
    [&body]() { (void)protocol::detail::requiredString(body, "missing", "field is required"); }, "field is required");
  expectInvalidArgument(
    [&body]() {
      (void)protocol::detail::requiredString(body, "blank", "field must be a string", "field must not be empty");
    },
    "field must not be empty");
  expectInvalidArgument(
    [&body]() { (void)protocol::detail::requiredString(body, "wrong_type", "field must be a string"); },
    "field must be a string");
}

TEST(PayloadHelpersTest, OptionalStringHandlesAbsentBlankNullAndInvalidValues)
{
  const nlohmann::json body = {
    {"blank", "   "},
    {"null_value", nullptr},
    {"trimmed", "  /camera/image  "},
    {"wrong_type", 125},
  };

  EXPECT_EQ(protocol::detail::optionalString(body, "missing", "field must be a string"), std::nullopt);
  EXPECT_EQ(protocol::detail::optionalString(body, "blank", "field must be a string"), std::nullopt);
  EXPECT_EQ(
    protocol::detail::optionalString(body, "null_value", "field must be a string", /*null_is_absent=*/true),
    std::nullopt);
  EXPECT_EQ(
    protocol::detail::optionalString(body, "trimmed", "field must be a string"),
    std::optional<std::string>("/camera/image"));

  expectInvalidArgument(
    [&body]() { (void)protocol::detail::optionalString(body, "null_value", "field must be a string"); },
    "field must be a string");
  expectInvalidArgument(
    [&body]() { (void)protocol::detail::optionalString(body, "wrong_type", "field must be a string"); },
    "field must be a string");
}

TEST(PayloadHelpersTest, ParseCdrRoundTripsEmptyAndBinaryPayloads)
{
  const nlohmann::json empty_body = {
    {"message", protocol::cdr::serialize(std::vector<std::uint8_t>{})},
  };
  EXPECT_TRUE(parsePayload(empty_body, protocol::cdr::Field::Message).empty());

  const std::vector<std::uint8_t> payload = {0x00, 0x01, 0x7f, 0x80, 0xff};
  const nlohmann::json binary_body = {
    {"message", protocol::cdr::serialize(payload)},
  };
  EXPECT_EQ(parsePayload(binary_body, protocol::cdr::Field::Message), payload);
}

TEST(PayloadHelpersTest, ParseCdrUsesRequestedOuterFieldName)
{
  const std::vector<std::uint8_t> payload = {0x01, 0x02, 0x03};
  const auto envelope = protocol::cdr::serialize(payload);

  EXPECT_EQ(parsePayload(nlohmann::json{{"request", envelope}}, protocol::cdr::Field::Request), payload);
}

TEST(PayloadHelpersTest, ParseCdrUsesRequestedOuterFieldNameInOuterEnvelopeErrors)
{
  expectInvalidArgument(
    []() {
      (void)protocol::cdr::parse(
        nlohmann::json{
          {"message",
           {
             {"content_type", "application/x-ros-cdr"},
             {"payload_base64", "AQID"},
           }},
        },
        protocol::cdr::Field::Request);
    },
    "request must be an object.");

  expectInvalidArgument(
    []() {
      (void)protocol::cdr::parse(
        nlohmann::json{
          {"request",
           {
             {"content_type", "application/json"},
             {"payload_base64", "AQID"},
           }},
        },
        protocol::cdr::Field::Request);
    },
    "request.content_type must be application/x-ros-cdr.");
}

TEST(PayloadHelpersTest, SerializeCdrEmitsCanonicalEnvelopeForEmptyAndPaddedPayloads)
{
  EXPECT_EQ(
    protocol::cdr::serialize(std::vector<std::uint8_t>{}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", ""},
    }));

  EXPECT_EQ(
    protocol::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", "AQI="},
    }));
}

TEST(PayloadHelpersTest, ParseCdrRejectsMissingOrNonObjectEnvelope)
{
  auto missing_message = makeMessageBody();
  missing_message.erase("message");
  expectInvalidArgument(
    [&missing_message]() { (void)protocol::cdr::parse(missing_message, protocol::cdr::Field::Message); },
    "message must be an object.");

  auto non_object_message = makeMessageBody();
  non_object_message["message"] = nlohmann::json::array({1, 2});
  expectInvalidArgument(
    [&non_object_message]() { (void)protocol::cdr::parse(non_object_message, protocol::cdr::Field::Message); },
    "message must be an object.");
}

TEST(PayloadHelpersTest, ParseCdrRejectsMissingOrMistypedNestedFields)
{
  auto missing_content_type = makeMessageBody();
  missing_content_type["message"].erase("content_type");
  expectInvalidArgument(
    [&missing_content_type]() { (void)protocol::cdr::parse(missing_content_type, protocol::cdr::Field::Message); },
    "content_type must be a string.");

  auto mistyped_payload_base64 = makeMessageBody();
  mistyped_payload_base64["message"]["payload_base64"] = false;
  expectInvalidArgument(
    [&mistyped_payload_base64]() {
      (void)protocol::cdr::parse(mistyped_payload_base64, protocol::cdr::Field::Message);
    },
    "payload_base64 must be a string.");

  auto missing_payload_base64 = makeMessageBody();
  missing_payload_base64["message"].erase("payload_base64");
  expectInvalidArgument(
    [&missing_payload_base64]() { (void)protocol::cdr::parse(missing_payload_base64, protocol::cdr::Field::Message); },
    "payload_base64 must be a string.");
}

TEST(PayloadHelpersTest, ParseCdrRejectsWrongContentTypeAndInvalidBase64Encodings)
{
  auto wrong_content_type = makeMessageBody();
  wrong_content_type["message"]["content_type"] = "application/json";
  expectInvalidArgument(
    [&wrong_content_type]() { (void)protocol::cdr::parse(wrong_content_type, protocol::cdr::Field::Message); },
    "message.content_type must be application/x-ros-cdr.");

  auto missing_padding = makeMessageBody();
  missing_padding["message"]["payload_base64"] = "AQI";
  expectInvalidArgument(
    [&missing_padding]() { (void)protocol::cdr::parse(missing_padding, protocol::cdr::Field::Message); },
    "payload_base64 must be padded standard base64.");

  auto invalid_encoding = makeMessageBody();
  invalid_encoding["message"]["payload_base64"] = "AQI?";
  expectInvalidArgument(
    [&invalid_encoding]() { (void)protocol::cdr::parse(invalid_encoding, protocol::cdr::Field::Message); },
    "payload_base64 is not valid base64.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
