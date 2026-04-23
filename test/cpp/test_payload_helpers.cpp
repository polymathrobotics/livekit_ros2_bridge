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
#include "wire/cdr.hpp"
#include "wire/detail/json_fields.hpp"

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

TEST(PayloadHelpersTest, ParseObjectPayloadRejectsInvalidJsonAndNonObjectRoot)
{
  expectInvalidArgumentMessage(
    []() { (void)wire::detail::parseObjectPayload("{", "payload JSON is invalid", "payload must be a JSON object"); },
    "payload JSON is invalid");
  expectInvalidArgumentMessage(
    []() {
      (void)wire::detail::parseObjectPayload("[1,2,3]", "payload JSON is invalid", "payload must be a JSON object");
    },
    "payload must be a JSON object");
}

TEST(PayloadHelpersTest, RequiredTrimmedStringFieldTrimsAndRejectsInvalidValues)
{
  const nlohmann::json body = {
    {"trimmed", "  /camera/image  "},
    {"blank", "   "},
    {"wrong_type", 42},
  };

  EXPECT_EQ(wire::detail::requiredTrimmedStringField(body, "trimmed", "field is required"), "/camera/image");

  expectInvalidArgumentMessage(
    [&body]() { (void)wire::detail::requiredTrimmedStringField(body, "missing", "field is required"); },
    "field is required");
  expectInvalidArgumentMessage(
    [&body]() {
      (void)wire::detail::requiredTrimmedStringField(
        body, "blank", "field must be a string", "field must not be empty");
    },
    "field must not be empty");
  expectInvalidArgumentMessage(
    [&body]() { (void)wire::detail::requiredTrimmedStringField(body, "wrong_type", "field must be a string"); },
    "field must be a string");
}

TEST(PayloadHelpersTest, OptionalTrimmedStringFieldHandlesAbsentBlankNullAndInvalidValues)
{
  const nlohmann::json body = {
    {"blank", "   "},
    {"null_value", nullptr},
    {"trimmed", "  /camera/image  "},
    {"wrong_type", 125},
  };

  EXPECT_EQ(wire::detail::optionalTrimmedStringField(body, "missing", "field must be a string"), std::nullopt);
  EXPECT_EQ(wire::detail::optionalTrimmedStringField(body, "blank", "field must be a string"), std::nullopt);
  EXPECT_EQ(wire::detail::optionalTrimmedStringField(body, "null_value", "field must be a string", true), std::nullopt);
  EXPECT_EQ(
    wire::detail::optionalTrimmedStringField(body, "trimmed", "field must be a string"),
    std::optional<std::string>("/camera/image"));

  expectInvalidArgumentMessage(
    [&body]() { (void)wire::detail::optionalTrimmedStringField(body, "null_value", "field must be a string"); },
    "field must be a string");
  expectInvalidArgumentMessage(
    [&body]() { (void)wire::detail::optionalTrimmedStringField(body, "wrong_type", "field must be a string"); },
    "field must be a string");
}

TEST(PayloadHelpersTest, RequiredTrimmedStringArrayFieldTrimsEntriesAndRejectsInvalidValues)
{
  const nlohmann::json body = {
    {"interface_types", {" sensor_msgs/msg/BatteryState ", "std_msgs/msg/String", "sensor_msgs/msg/BatteryState "}},
  };

  EXPECT_EQ(
    wire::detail::requiredTrimmedStringArrayField(body, "interface_types"),
    (std::vector<std::string>{
      "sensor_msgs/msg/BatteryState",
      "std_msgs/msg/String",
      "sensor_msgs/msg/BatteryState",
    }));

  const nlohmann::json missing = {
    {"other_field", nlohmann::json::array()},
  };
  expectInvalidArgumentMessage(
    [&missing]() { (void)wire::detail::requiredTrimmedStringArrayField(missing, "interface_types"); },
    "interface_types must be an array");

  const nlohmann::json empty_array = {
    {"interface_types", nlohmann::json::array()},
  };
  expectInvalidArgumentMessage(
    [&empty_array]() { (void)wire::detail::requiredTrimmedStringArrayField(empty_array, "interface_types"); },
    "interface_types must not be empty");

  const nlohmann::json blank_entry = {
    {"interface_types", {"std_msgs/msg/String", "   "}},
  };
  expectInvalidArgumentMessage(
    [&blank_entry]() { (void)wire::detail::requiredTrimmedStringArrayField(blank_entry, "interface_types"); },
    "interface_types entries must not be empty");

  const nlohmann::json wrong_entry_type = {
    {"interface_types", {"std_msgs/msg/String", false}},
  };
  expectInvalidArgumentMessage(
    [&wrong_entry_type]() { (void)wire::detail::requiredTrimmedStringArrayField(wrong_entry_type, "interface_types"); },
    "interface_types entries must be strings");
}

TEST(PayloadHelpersTest, ParseCdrRoundTripsEmptyAndBinaryPayloads)
{
  const nlohmann::json empty_body = {
    {"message", wire::cdr::serialize(std::vector<std::uint8_t>{})},
  };
  EXPECT_TRUE(wire::cdr::parse(empty_body, "message").empty());

  const std::vector<std::uint8_t> payload = {0x00, 0x01, 0x7f, 0x80, 0xff};
  const nlohmann::json binary_body = {
    {"message", wire::cdr::serialize(payload)},
  };
  EXPECT_EQ(wire::cdr::parse(binary_body, "message"), payload);
}

TEST(PayloadHelpersTest, ParseCdrUsesRequestedOuterFieldName)
{
  const std::vector<std::uint8_t> payload = {0x01, 0x02, 0x03};
  const auto envelope = wire::cdr::serialize(payload);

  EXPECT_EQ(wire::cdr::parse(nlohmann::json{{"request", envelope}}, "request"), payload);
}

TEST(PayloadHelpersTest, ParseCdrUsesRequestedOuterFieldNameInOuterEnvelopeErrors)
{
  expectInvalidArgumentMessage(
    []() {
      (void)wire::cdr::parse(
        nlohmann::json{
          {"message",
           {
             {"content_type", "application/x-ros-cdr"},
             {"payload_base64", "AQID"},
           }},
        },
        "request");
    },
    "request must be an object.");

  expectInvalidArgumentMessage(
    []() {
      (void)wire::cdr::parse(
        nlohmann::json{
          {"request",
           {
             {"content_type", "application/json"},
             {"payload_base64", "AQID"},
           }},
        },
        "request");
    },
    "request.content_type must be application/x-ros-cdr.");
}

TEST(PayloadHelpersTest, SerializeCdrEmitsCanonicalEnvelopeForEmptyAndPaddedPayloads)
{
  EXPECT_EQ(
    wire::cdr::serialize({}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", ""},
    }));

  EXPECT_EQ(
    wire::cdr::serialize({0x01, 0x02}),
    (nlohmann::json{
      {"content_type", "application/x-ros-cdr"},
      {"payload_base64", "AQI="},
    }));
}

TEST(PayloadHelpersTest, ParseCdrRejectsMissingOrNonObjectEnvelope)
{
  auto missing_message = makeMessageBody();
  missing_message.erase("message");
  expectInvalidArgumentMessage(
    [&missing_message]() { (void)wire::cdr::parse(missing_message, "message"); }, "message must be an object.");

  auto non_object_message = makeMessageBody();
  non_object_message["message"] = nlohmann::json::array({1, 2});
  expectInvalidArgumentMessage(
    [&non_object_message]() { (void)wire::cdr::parse(non_object_message, "message"); }, "message must be an object.");
}

TEST(PayloadHelpersTest, ParseCdrRejectsMissingOrMistypedNestedFields)
{
  auto missing_content_type = makeMessageBody();
  missing_content_type["message"].erase("content_type");
  expectInvalidArgumentMessage(
    [&missing_content_type]() { (void)wire::cdr::parse(missing_content_type, "message"); },
    "content_type must be a string.");

  auto mistyped_payload_base64 = makeMessageBody();
  mistyped_payload_base64["message"]["payload_base64"] = false;
  expectInvalidArgumentMessage(
    [&mistyped_payload_base64]() { (void)wire::cdr::parse(mistyped_payload_base64, "message"); },
    "payload_base64 must be a string.");

  auto missing_payload_base64 = makeMessageBody();
  missing_payload_base64["message"].erase("payload_base64");
  expectInvalidArgumentMessage(
    [&missing_payload_base64]() { (void)wire::cdr::parse(missing_payload_base64, "message"); },
    "payload_base64 must be a string.");
}

TEST(PayloadHelpersTest, ParseCdrRejectsWrongContentTypeAndInvalidBase64Encodings)
{
  auto wrong_content_type = makeMessageBody();
  wrong_content_type["message"]["content_type"] = "application/json";
  expectInvalidArgumentMessage(
    [&wrong_content_type]() { (void)wire::cdr::parse(wrong_content_type, "message"); },
    "message.content_type must be application/x-ros-cdr.");

  auto missing_padding = makeMessageBody();
  missing_padding["message"]["payload_base64"] = "AQI";
  expectInvalidArgumentMessage(
    [&missing_padding]() { (void)wire::cdr::parse(missing_padding, "message"); },
    "payload_base64 must be padded standard base64.");

  auto invalid_encoding = makeMessageBody();
  invalid_encoding["message"]["payload_base64"] = "AQI?";
  expectInvalidArgumentMessage(
    [&invalid_encoding]() { (void)wire::cdr::parse(invalid_encoding, "message"); },
    "payload_base64 is not valid base64.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
