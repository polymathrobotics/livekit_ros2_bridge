// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/echo_once_json.hpp"
#include "protocol/validation_error.hpp"
#include "support/protocol_test_support.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::expectInvalidArgument;

EchoOnceRequest parsePayload(const std::string & payload)
{
  return protocol::echo_once::parse(payload);
}

void expectParseError(const std::string & payload, const char * expected_message, const char * expected_field)
{
  expectInvalidArgument([&payload]() { (void)parsePayload(payload); }, expected_message, expected_field);
}

TEST(EchoOncePayloadsTest, ParseAcceptsTopicKindAndExpandsName)
{
  const auto request = parsePayload(R"({"kind":"topic","name":"/map"})");
  EXPECT_EQ(request.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(request.name, "/map");
}

TEST(EchoOncePayloadsTest, ParseExpandsRelativeTopicNameToGlobal)
{
  const auto request = parsePayload(R"({"kind":"topic","name":"map"})");
  EXPECT_EQ(request.name, "/map");
}

TEST(EchoOncePayloadsTest, ParseTrimsKindAndName)
{
  const auto request = parsePayload(R"({"kind":"  topic  ","name":"  /map  "})");
  EXPECT_EQ(request.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(request.name, "/map");
}

TEST(EchoOncePayloadsTest, ParseRejectsUnsupportedKind)
{
  // `other_video` is a valid subscription kind but is not supported by the echo-once primitive;
  // it must fail loudly rather than silently returning `none`.
  expectParseError(R"({"kind":"other_video","name":"/map"})", "echo-once 'kind' must be 'topic'", "kind");
  expectParseError(R"({"kind":"future_kind","name":"/map"})", "echo-once 'kind' must be 'topic'", "kind");
}

TEST(EchoOncePayloadsTest, ParseRejectsMissingOrNonStringKind)
{
  expectParseError(R"({"name":"/map"})", "echo-once 'kind' must be a string", "kind");
  expectParseError(R"({"kind":1,"name":"/map"})", "echo-once 'kind' must be a string", "kind");
}

TEST(EchoOncePayloadsTest, ParseRejectsMissingOrNonStringName)
{
  expectParseError(R"({"kind":"topic"})", "echo-once 'name' must be a string", "name");
  expectParseError(R"({"kind":"topic","name":42})", "echo-once 'name' must be a string", "name");
}

TEST(EchoOncePayloadsTest, ParseRejectsInvalidTopicNameAsNameValidationError)
{
  try {
    (void)parsePayload(R"({"kind":"topic","name":"bad name with spaces"})");
    ADD_FAILURE() << "Expected a validation error for an invalid topic name";
  } catch (const protocol::ValidationError & error) {
    EXPECT_EQ(error.field(), "name");
  }
}

TEST(EchoOncePayloadsTest, ParseRejectsMalformedJsonAsPayloadError)
{
  expectParseError("not json", "Invalid JSON in echo-once request", "payload");
}

TEST(EchoOncePayloadsTest, ParseRejectsNonObjectPayload)
{
  expectParseError(R"(["topic","/map"])", "Echo-once request must be a JSON object", "payload");
}

TEST(EchoOncePayloadsTest, SerializeEmitsSent)
{
  const auto body = nlohmann::json::parse(protocol::echo_once::serialize(EchoOnceResult::Sent));
  EXPECT_EQ(body, (nlohmann::json{{"result", "sent"}}));
}

TEST(EchoOncePayloadsTest, SerializeEmitsNone)
{
  const auto body = nlohmann::json::parse(protocol::echo_once::serialize(EchoOnceResult::None));
  EXPECT_EQ(body, (nlohmann::json{{"result", "none"}}));
}

}  // namespace
}  // namespace livekit_ros2_bridge
