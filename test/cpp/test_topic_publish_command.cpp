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

#include <array>
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

TEST(TopicPublishCommandTest, RejectsInvalidJson)
{
  EXPECT_THROW(parseTopicPublishCommand(toBytes("{")), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsNonObject)
{
  EXPECT_THROW(parseTopicPublishCommand(toBytes("[1,2,3]")), std::invalid_argument);
}

TEST(TopicPublishCommandTest, RejectsMissingOrInvalidTopic)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json{
      {"interface_type", "std_msgs/msg/String"},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },

    nlohmann::json{
      {"topic", ""},
      {"interface_type", "std_msgs/msg/String"},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },

    nlohmann::json{
      {"topic", 123},
      {"interface_type", "std_msgs/msg/String"},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },
  };

  for (const auto & command_payload : payloads) {
    EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
  }
}

TEST(TopicPublishCommandTest, RejectsMissingOrInvalidInterfaceType)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json{
      {"topic", "/chatter"},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },

    nlohmann::json{
      {"topic", "/chatter"},
      {"interface_type", ""},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },

    nlohmann::json{
      {"topic", "/chatter"},
      {"interface_type", false},
      {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01})},
    },
  };

  for (const auto & command_payload : payloads) {
    EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
  }
}

TEST(TopicPublishCommandTest, RejectsMissingOrMalformedMessage)
{
  const std::array<nlohmann::json, 2> payloads{
    nlohmann::json{
      {"topic", "/chatter"},
      {"interface_type", "std_msgs/msg/String"},
    },

    nlohmann::json{
      {"topic", "/chatter"},
      {"interface_type", "std_msgs/msg/String"},
      {"message", nlohmann::json::array({1, 2})},
    },
  };

  for (const auto & command_payload : payloads) {
    EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
  }
}

TEST(TopicPublishCommandTest, RejectsEmptyMessagePayload)
{
  auto command_payload = makeValidPayload();
  command_payload["message"] = {
    {"content_type", "application/x-ros-cdr"},
    {"payload_base64", ""},
  };

  EXPECT_THROW(parseCommand(command_payload), std::invalid_argument);
}

}  // namespace

}  // namespace livekit_ros2_bridge
