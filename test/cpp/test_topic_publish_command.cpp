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
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "topic_publish_command.hpp"

namespace livekit_ros2_bridge
{

namespace
{

static std::vector<std::uint8_t> toBytes(const std::string & s)
{
  return std::vector<std::uint8_t>(s.begin(), s.end());
}

TopicPublishCommand parseCommand(const nlohmann::json & command_payload)
{
  return parseTopicPublishCommand(toBytes(command_payload.dump()));
}

nlohmann::json makeValidPayload()
{
  return nlohmann::json{
    {"topic", "/chatter"},
    {"interface_type", "std_msgs/msg/String"},
    {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
  };
}

TEST(TopicPublishCommandTest, ParsesValidCommandAndNormalizesFields)
{
  auto command_payload = makeValidPayload();
  command_payload["topic"] = " //camera///image/ ";
  command_payload["interface_type"] = "  std_msgs/msg/String  ";

  const auto command = parseCommand(command_payload);

  EXPECT_EQ(command.topic, "/camera/image");
  EXPECT_EQ(command.interface_type, "std_msgs/msg/String");
  EXPECT_EQ(command.cdr_payload, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
}

TEST(TopicPublishCommandTest, AcceptsRootTopicAfterNormalization)
{
  auto command_payload = makeValidPayload();
  command_payload["topic"] = "  ////  ";

  const auto command = parseCommand(command_payload);

  EXPECT_EQ(command.topic, "/");
}

TEST(TopicPublishCommandTest, NormalizesRelativeTopicNamesAndPreservesBinaryPayload)
{
  const std::vector<std::uint8_t> payload = {0x00, 0x01, 0x7f, 0x80, 0xff};

  auto command_payload = makeValidPayload();
  command_payload["topic"] = "  battery/cmd  ";
  command_payload["message"] = serializeCdrPayload(payload);

  const auto command = parseCommand(command_payload);

  EXPECT_EQ(command.topic, "/battery/cmd");
  EXPECT_EQ(command.cdr_payload, payload);
}

TEST(TopicPublishCommandTest, RejectsInvalidJsonAndNonObjectRoot)
{
  EXPECT_THROW(parseTopicPublishCommand(toBytes("{")), std::invalid_argument);
  EXPECT_THROW(parseTopicPublishCommand(toBytes("[1,2,3]")), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingBlankOrNonStringTopicField)
{
  auto command_payload = makeValidPayload();
  command_payload.erase("topic");
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);

  command_payload = makeValidPayload();
  command_payload["topic"] = "   ";
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);

  command_payload = makeValidPayload();
  command_payload["topic"] = 123;
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingBlankOrNonStringInterfaceTypeField)
{
  auto command_payload = makeValidPayload();
  command_payload.erase("interface_type");
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);

  command_payload = makeValidPayload();
  command_payload["interface_type"] = "   ";
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);

  command_payload = makeValidPayload();
  command_payload["interface_type"] = false;
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingOrNonObjectMessageField)
{
  auto command_payload = makeValidPayload();
  command_payload.erase("message");
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);

  command_payload = makeValidPayload();
  command_payload["message"] = "not-an-object";
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

// TODO: Keep exhaustive CDR envelope structure/content-type/base64 cases in
// test_payload_helpers.cpp; this file only needs a representative delegated
// failure plus the command-specific non-empty payload rule.
TEST(TopicPublishCommandTest, RejectsUnsupportedMessageContentType)
{
  auto command_payload = makeValidPayload();
  command_payload["message"] = {
    {"content_type", "application/json"},
    {"payload_base64", "AQID"},
  };
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsEmptyMessagePayload)
{
  auto command_payload = makeValidPayload();
  command_payload["message"] = serializeCdrPayload(std::vector<std::uint8_t>{});
  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

}  // namespace

}  // namespace livekit_ros2_bridge
