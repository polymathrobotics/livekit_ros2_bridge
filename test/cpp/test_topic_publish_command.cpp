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

TopicPublishCommand parseCommand(const nlohmann::json & body)
{
  const auto text = body.dump();
  return parseTopicPublishCommand(std::vector<std::uint8_t>(text.begin(), text.end()));
}

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

nlohmann::json makeBody()
{
  return nlohmann::json{
    {"topic", "/chatter"},
    {"interface_type", "std_msgs/msg/String"},
    {"message", cdr_payload::serialize(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
  };
}

TEST(TopicPublishCommandTest, ParsesValidCommandAndNormalizesFields)
{
  auto body = makeBody();
  body["topic"] = " //camera///image/ ";
  body["interface_type"] = "  std_msgs/msg/String  ";

  const auto command = parseCommand(body);

  EXPECT_EQ(command.topic, "/camera/image");
  EXPECT_EQ(command.interface_type, "std_msgs/msg/String");
  EXPECT_EQ(command.cdr, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
}

TEST(TopicPublishCommandTest, AcceptsRootTopicAfterNormalization)
{
  auto body = makeBody();
  body["topic"] = "  ////  ";

  const auto command = parseCommand(body);

  EXPECT_EQ(command.topic, "/");
}

TEST(TopicPublishCommandTest, NormalizesRelativeTopicNamesAndPreservesBinaryPayload)
{
  const std::vector<std::uint8_t> cdr = {0x00, 0x01, 0x7f, 0x80, 0xff};

  auto body = makeBody();
  body["topic"] = "  battery/cmd  ";
  body["message"] = cdr_payload::serialize(cdr);

  const auto command = parseCommand(body);

  EXPECT_EQ(command.topic, "/battery/cmd");
  EXPECT_EQ(command.cdr, cdr);
}

TEST(TopicPublishCommandTest, RejectsInvalidJsonAndNonObjectRoot)
{
  EXPECT_THROW(parseTopicPublishCommand(std::vector<std::uint8_t>{'{'}), std::invalid_argument);
  EXPECT_THROW(
    parseTopicPublishCommand(std::vector<std::uint8_t>{'[', '1', ',', '2', ',', '3', ']'}), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingBlankOrNonStringTopicField)
{
  auto body = makeBody();
  body.erase("topic");
  expectInvalidArgumentMessage(
    [&body]() { (void)parseCommand(body); }, "Publish command requires a string 'topic' field.");

  body = makeBody();
  body["topic"] = "   ";
  expectInvalidArgumentMessage(
    [&body]() { (void)parseCommand(body); }, "Publish command requires a non-empty 'topic' field.");

  body = makeBody();
  body["topic"] = 123;
  expectInvalidArgumentMessage(
    [&body]() { (void)parseCommand(body); }, "Publish command requires a string 'topic' field.");
}

TEST(TopicPublishCommandTest, RejectsMissingBlankOrNonStringInterfaceTypeField)
{
  auto body = makeBody();
  body.erase("interface_type");
  EXPECT_THROW(parseCommand(body), std::invalid_argument);

  body = makeBody();
  body["interface_type"] = "   ";
  EXPECT_THROW(parseCommand(body), std::invalid_argument);

  body = makeBody();
  body["interface_type"] = false;
  EXPECT_THROW(parseCommand(body), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingOrNonObjectMessageField)
{
  auto body = makeBody();
  body.erase("message");
  EXPECT_THROW(parseCommand(body), std::invalid_argument);

  body = makeBody();
  body["message"] = "not-an-object";
  EXPECT_THROW(parseCommand(body), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsUnsupportedMessageContentType)
{
  auto body = makeBody();
  body["message"] = {
    {"content_type", "application/json"},
    {"payload_base64", "AQID"},
  };
  EXPECT_THROW(parseCommand(body), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsEmptyMessagePayload)
{
  auto body = makeBody();
  body["message"] = cdr_payload::serialize(std::vector<std::uint8_t>{});
  expectInvalidArgumentMessage(
    [&body]() { (void)parseCommand(body); }, "Publish command requires a non-empty message.payload_base64 field.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
