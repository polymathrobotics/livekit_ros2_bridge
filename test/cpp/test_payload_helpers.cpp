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
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "payloads/json_object_parser.hpp"

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

nlohmann::json makeValidCdrBody()
{
  return nlohmann::json{
    {"message",
     {
       {"content_type", "application/x-ros-cdr"},
       {"payload_base64", "AQID"},
     }},
  };
}

TEST(PayloadHelpersTest, ParseJsonObjectRejectsInvalidJsonAndNonObjectRoot)
{
  expectInvalidArgumentMessage(
    []() { (void)parseJsonObject("{", "payload JSON is invalid", "payload must be a JSON object"); },
    "payload JSON is invalid");
  expectInvalidArgumentMessage(
    []() { (void)parseJsonObject("[1,2,3]", "payload JSON is invalid", "payload must be a JSON object"); },
    "payload must be a JSON object");
}

TEST(PayloadHelpersTest, ParseRequiredNonEmptyTrimmedStringTrimsValidValueAndRejectsInvalidInputs)
{
  EXPECT_EQ(
    parseRequiredNonEmptyTrimmedString(
      nlohmann::json("  std_msgs/msg/String  "), "field must be a string", "field must not be empty"),
    "std_msgs/msg/String");

  expectInvalidArgumentMessage(
    []() {
      (void)parseRequiredNonEmptyTrimmedString(nlohmann::json(42), "field must be a string", "field must not be empty");
    },
    "field must be a string");
  expectInvalidArgumentMessage(
    []() {
      (void)parseRequiredNonEmptyTrimmedString(
        nlohmann::json("   "), "field must be a string", "field must not be empty");
    },
    "field must not be empty");
}

TEST(PayloadHelpersTest, ParseRequiredTrimmedStringFieldReadsPresentFieldAndRejectsMissing)
{
  const nlohmann::json body = {
    {"trimmed", "  /camera/image  "},
  };

  EXPECT_EQ(parseRequiredNonEmptyTrimmedStringField(body, "trimmed", "field is required"), "/camera/image");

  expectInvalidArgumentMessage(
    [&body]() { (void)parseRequiredNonEmptyTrimmedStringField(body, "missing", "field is required"); },
    "field is required");
}

TEST(PayloadHelpersTest, ParseCdrPayloadRoundTripsEmptyAndBinaryPayloads)
{
  const nlohmann::json empty_body = {
    {"message", serializeCdrPayload(std::vector<std::uint8_t>{})},
  };
  EXPECT_TRUE(parseCdrPayload(empty_body, "message").empty());

  const std::vector<std::uint8_t> payload = {0x00, 0x01, 0x7f, 0x80, 0xff};
  const nlohmann::json binary_body = {
    {"message", serializeCdrPayload(payload)},
  };
  EXPECT_EQ(parseCdrPayload(binary_body, "message"), payload);
}

TEST(PayloadHelpersTest, ParseOptionalTrimmedStringFieldHandlesAbsentAndInvalidValues)
{
  const nlohmann::json body = {
    {"blank", "   "},
    {"null_value", nullptr},
    {"trimmed", "  /camera/image  "},
    {"wrong_type", 125},
  };

  EXPECT_EQ(parseOptionalNonEmptyTrimmedStringField(body, "missing", "field must be a string"), std::nullopt);
  EXPECT_EQ(parseOptionalNonEmptyTrimmedStringField(body, "blank", "field must be a string"), std::nullopt);
  EXPECT_EQ(parseOptionalNonEmptyTrimmedStringField(body, "null_value", "field must be a string", true), std::nullopt);
  EXPECT_EQ(
    parseOptionalNonEmptyTrimmedStringField(body, "trimmed", "field must be a string"),
    std::optional<std::string>("/camera/image"));

  expectInvalidArgumentMessage(
    [&body]() { (void)parseOptionalNonEmptyTrimmedStringField(body, "null_value", "field must be a string"); },
    "field must be a string");
  expectInvalidArgumentMessage(
    [&body]() { (void)parseOptionalNonEmptyTrimmedStringField(body, "wrong_type", "field must be a string"); },
    "field must be a string");
}

TEST(PayloadHelpersTest, SerializeCdrPayloadEmitsCanonicalEnvelopeForEmptyAndPaddedPayloads)
{
  EXPECT_EQ(
    serializeCdrPayload({}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", ""},
    }));

  EXPECT_EQ(
    serializeCdrPayload({0x01, 0x02}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", "AQI="},
    }));
}

TEST(PayloadHelpersTest, ParseCdrPayloadRejectsMissingOrNonObjectEnvelope)
{
  auto missing_message = makeValidCdrBody();
  missing_message.erase("message");
  expectInvalidArgumentMessage(
    [&missing_message]() { (void)parseCdrPayload(missing_message, "message"); }, "message must be an object.");

  auto non_object_message = makeValidCdrBody();
  non_object_message["message"] = nlohmann::json::array({1, 2});
  expectInvalidArgumentMessage(
    [&non_object_message]() { (void)parseCdrPayload(non_object_message, "message"); }, "message must be an object.");
}

TEST(PayloadHelpersTest, ParseCdrPayloadRejectsMissingOrMistypedNestedFields)
{
  auto missing_content_type = makeValidCdrBody();
  missing_content_type["message"].erase("content_type");
  expectInvalidArgumentMessage(
    [&missing_content_type]() { (void)parseCdrPayload(missing_content_type, "message"); },
    "content_type must be a string.");

  auto mistyped_payload_base64 = makeValidCdrBody();
  mistyped_payload_base64["message"]["payload_base64"] = false;
  expectInvalidArgumentMessage(
    [&mistyped_payload_base64]() { (void)parseCdrPayload(mistyped_payload_base64, "message"); },
    "payload_base64 must be a string.");
}

TEST(PayloadHelpersTest, ParseCdrPayloadRejectsWrongContentTypeAndInvalidBase64Encodings)
{
  auto wrong_content_type = makeValidCdrBody();
  wrong_content_type["message"]["content_type"] = "application/json";
  expectInvalidArgumentMessage(
    [&wrong_content_type]() { (void)parseCdrPayload(wrong_content_type, "message"); },
    "message.content_type must be application/x-ros-cdr.");

  auto missing_padding = makeValidCdrBody();
  missing_padding["message"]["payload_base64"] = "AQI";
  expectInvalidArgumentMessage(
    [&missing_padding]() { (void)parseCdrPayload(missing_padding, "message"); },
    "payload_base64 must be padded standard base64.");

  auto invalid_encoding = makeValidCdrBody();
  invalid_encoding["message"]["payload_base64"] = "AQI?";
  expectInvalidArgumentMessage(
    [&invalid_encoding]() { (void)parseCdrPayload(invalid_encoding, "message"); },
    "payload_base64 is not valid base64.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
