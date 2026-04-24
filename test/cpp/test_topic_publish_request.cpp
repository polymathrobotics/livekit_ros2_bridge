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
#include "protocol/cdr.hpp"
#include "protocol/topic_publish.hpp"
#include "protocol/topic_publish_json.hpp"
#include "protocol_test_support.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TopicPublishRequest parse(const nlohmann::json & body)
{
  const auto text = body.dump();
  return protocol::topic_publish::parse(std::vector<std::uint8_t>(text.begin(), text.end()));
}

using test_support::expectInvalidArgument;

nlohmann::json makeBody()
{
  return nlohmann::json{
    {"topic", "/chatter"},
    {"interface_type", "std_msgs/msg/String"},
    {"message", protocol::cdr::serialize(std::vector<std::uint8_t>{0x01, 0x02, 0x03})},
  };
}

TEST(TopicPublishRequestTest, ParsesValidRequestAndNormalizesFields)
{
  auto body = makeBody();
  body["topic"] = " //camera///image/ ";
  body["interface_type"] = "  std_msgs/msg/String  ";

  const auto request = parse(body);

  EXPECT_EQ(request.ros_topic, "/camera/image");
  EXPECT_EQ(request.interface_type, "std_msgs/msg/String");
  EXPECT_EQ(request.cdr, (std::vector<std::uint8_t>{0x01, 0x02, 0x03}));
}

TEST(TopicPublishRequestTest, AcceptsRootTopicAfterNormalization)
{
  auto body = makeBody();
  body["topic"] = "  ////  ";

  const auto request = parse(body);

  EXPECT_EQ(request.ros_topic, "/");
}

TEST(TopicPublishRequestTest, NormalizesRelativeTopicNamesAndPreservesBinaryPayload)
{
  const std::vector<std::uint8_t> cdr = {0x00, 0x01, 0x7f, 0x80, 0xff};

  auto body = makeBody();
  body["topic"] = "  battery/cmd  ";
  body["message"] = protocol::cdr::serialize(cdr);

  const auto request = parse(body);

  EXPECT_EQ(request.ros_topic, "/battery/cmd");
  EXPECT_EQ(request.cdr, cdr);
}

TEST(TopicPublishRequestTest, RejectsInvalidJsonAndNonObjectRoot)
{
  EXPECT_THROW(protocol::topic_publish::parse(std::vector<std::uint8_t>{'{'}), std::invalid_argument);
  EXPECT_THROW(
    protocol::topic_publish::parse(std::vector<std::uint8_t>{'[', '1', ',', '2', ',', '3', ']'}),
    std::invalid_argument);
}

TEST(TopicPublishRequestTest, RejectsMissingOrBlankTopicField)
{
  auto body = makeBody();
  body.erase("topic");
  expectInvalidArgument([&body]() { (void)parse(body); }, "Publish request requires a string 'topic' field.");

  body = makeBody();
  body["topic"] = "   ";
  expectInvalidArgument([&body]() { (void)parse(body); }, "Publish request requires a non-empty 'topic' field.");
}

TEST(TopicPublishRequestTest, RejectsMissingInterfaceTypeField)
{
  auto body = makeBody();
  body.erase("interface_type");
  expectInvalidArgument(
    [&body]() { (void)parse(body); }, "Publish request requires a non-empty 'interface_type' field.");
}

TEST(TopicPublishRequestTest, RejectsEmptyMessagePayload)
{
  auto body = makeBody();
  body["message"] = protocol::cdr::serialize(std::vector<std::uint8_t>{});
  expectInvalidArgument(
    [&body]() { (void)parse(body); }, "Publish request requires a non-empty message.payload_base64 field.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
