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
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/stream_control_payloads.hpp"

namespace livekit_ros2_bridge
{
namespace
{

void expectDemand(
  const nlohmann::json & body,
  SubscriptionTargetKind expected_kind,
  const std::string & expected_name,
  std::optional<int> expected_interval_ms)
{
  const auto heartbeat = stream_control_payloads::parseSubscriptionHeartbeat(body);
  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);

  const SubscriptionDemand & demand = heartbeat.subscriptions[0];
  EXPECT_EQ(demand.target.kind, expected_kind);
  EXPECT_EQ(demand.target.name, expected_name);
  EXPECT_EQ(demand.preferred_interval_ms, expected_interval_ms);
}

void expectParseError(const nlohmann::json & body)
{
  EXPECT_THROW((void)stream_control_payloads::parseSubscriptionHeartbeat(body), std::invalid_argument);
}

SubscriptionStatus makeStatus(
  SubscriptionTargetKind target_kind,
  std::string target_name,
  SubscriptionDeliveryKind delivery_kind,
  std::string track_name)
{
  SubscriptionStatus status;
  status.target = {target_kind, std::move(target_name)};
  status.delivery_kind = delivery_kind;
  status.track_name = std::move(track_name);
  return status;
}

TEST(StreamControlPayloadsTest, ParseHeartbeatNormalizesTargetsAndIntervals)
{
  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":" topic ","name":" battery ","delivery_preferences":{"interval_ms":125},"accepts":"application/x-ros-cdr"}]})"),
    SubscriptionTargetKind::Topic,
    "/battery",
    125);

  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":" battery ","delivery_preferences":{"interval_ms":125},"accepts":"application/x-ros-cdr"}]})"),
    SubscriptionTargetKind::Topic,
    "/battery",
    125);

  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"configured_source","name":" front_camera ","delivery_preferences":{"interval_ms":125}}]})"),
    SubscriptionTargetKind::ConfiguredSource,
    "front_camera",
    125);

  expectDemand(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/camera"}]})"),
    SubscriptionTargetKind::Topic,
    "/camera",
    std::nullopt);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatParsesOptionalSessionIdAndRejectsMistypedValues)
{
  const auto trimmed = stream_control_payloads::parseSubscriptionHeartbeat(nlohmann::json::parse(R"({
      "session_id":"  session-1  ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })"));
  ASSERT_TRUE(trimmed.session_id.has_value());
  EXPECT_EQ(*trimmed.session_id, "session-1");

  const auto missing = stream_control_payloads::parseSubscriptionHeartbeat(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/battery"}]})"));
  EXPECT_EQ(missing.session_id, std::nullopt);

  const auto blank = stream_control_payloads::parseSubscriptionHeartbeat(nlohmann::json::parse(R"({
      "session_id":"   ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })"));
  EXPECT_EQ(blank.session_id, std::nullopt);

  const auto null_id = stream_control_payloads::parseSubscriptionHeartbeat(nlohmann::json::parse(R"({
      "session_id":null,
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })"));
  EXPECT_EQ(null_id.session_id, std::nullopt);

  expectParseError(nlohmann::json::parse(R"({
    "session_id":125,
    "subscriptions":[{"kind":"topic","name":"/battery"}]
  })"));
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsMissingOrMalformedSubscriptions)
{
  expectParseError(nlohmann::json::parse(R"({})"));
  expectParseError(nlohmann::json::parse(R"({"subscriptions":{"topic":"/battery"}})"));
  expectParseError(nlohmann::json::parse(R"({"subscriptions":["/battery"]})"));
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsMissingOrNonStringTargetFields)
{
  expectParseError(nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic"}]})"));
  expectParseError(nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":123}]})"));
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsBlankOrUnsupportedTargets)
{
  expectParseError(nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"   "}]})"));
  expectParseError(nlohmann::json::parse(R"({"subscriptions":[{"kind":"configured_source","name":"   "}]})"));
  expectParseError(nlohmann::json::parse(R"({"subscriptions":[{"kind":"service","name":"/battery"}]})"));
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsInvalidIntervalTypes)
{
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":125}]})"));
  expectParseError(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":{"interval_ms":"125"}}]})"));
}

TEST(StreamControlPayloadsTest, ParseHeartbeatClampsOutOfRangeIntervals)
{
  const auto expectClampedInterval = [](std::int64_t raw_interval_ms, int expected_interval_ms) {
    const nlohmann::json body = {
      {"subscriptions",
       {{{"kind", "topic"}, {"name", "/lidar"}, {"delivery_preferences", {{"interval_ms", raw_interval_ms}}}}}}};

    expectDemand(body, SubscriptionTargetKind::Topic, "/lidar", expected_interval_ms);
  };

  expectClampedInterval(std::numeric_limits<std::int64_t>::max(), std::numeric_limits<int>::max());
  expectClampedInterval(std::numeric_limits<std::int64_t>::min(), std::numeric_limits<int>::min());

  const nlohmann::json unsigned_body = {
    {"subscriptions",
     {{{"kind", "topic"},
       {"name", "/lidar"},
       {"delivery_preferences", {{"interval_ms", std::numeric_limits<std::uint64_t>::max()}}}}}}};
  expectDemand(unsigned_body, SubscriptionTargetKind::Topic, "/lidar", std::numeric_limits<int>::max());
}

TEST(StreamControlPayloadsTest, ParseHeartbeatCoalescesDuplicateTopicsUsingMinimumInterval)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":25}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto heartbeat = stream_control_payloads::parseSubscriptionHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);
  EXPECT_EQ(heartbeat.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(heartbeat.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatCoalescesDuplicateConfiguredSourcesUsingTrimmedName)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"configured_source","name":" front_camera ","delivery_preferences":{"interval_ms":125}},
      {"kind":" configured_source ","name":"front_camera","delivery_preferences":{"interval_ms":25}}
    ]})");
  const auto heartbeat = stream_control_payloads::parseSubscriptionHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);
  EXPECT_EQ(heartbeat.subscriptions[0].target.kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(heartbeat.subscriptions[0].target.name, "front_camera");
  EXPECT_EQ(heartbeat.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatTreatsZeroIntervalAsNoPreferenceDuringCoalescing)
{
  const auto zero_then_non_zero = stream_control_payloads::parseSubscriptionHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":0}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})"));
  ASSERT_EQ(zero_then_non_zero.subscriptions.size(), 1U);
  EXPECT_EQ(zero_then_non_zero.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(zero_then_non_zero.subscriptions[0].preferred_interval_ms, 125);

  const auto non_zero_then_zero = stream_control_payloads::parseSubscriptionHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":0}}
    ]})"));
  ASSERT_EQ(non_zero_then_zero.subscriptions.size(), 1U);
  EXPECT_EQ(non_zero_then_zero.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(non_zero_then_zero.subscriptions[0].preferred_interval_ms, 125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatTreatsEmptyDeliveryPreferencesAsNoPreferenceDuringCoalescing)
{
  const auto empty_then_non_zero = stream_control_payloads::parseSubscriptionHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})"));
  ASSERT_EQ(empty_then_non_zero.subscriptions.size(), 1U);
  EXPECT_EQ(empty_then_non_zero.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(empty_then_non_zero.subscriptions[0].preferred_interval_ms, 125);

  const auto non_zero_then_empty = stream_control_payloads::parseSubscriptionHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{}}
    ]})"));
  ASSERT_EQ(non_zero_then_empty.subscriptions.size(), 1U);
  EXPECT_EQ(non_zero_then_empty.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(non_zero_then_empty.subscriptions[0].preferred_interval_ms, 125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatKeepsDistinctSubscriptionKeysSeparate)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"topic","name":"/camera/front","delivery_preferences":{"interval_ms":25}},
      {"kind":"configured_source","name":"/camera/front","delivery_preferences":{"interval_ms":125}},
      {"kind":"configured_source","name":"front_camera","delivery_preferences":{"interval_ms":25}},
      {"kind":"configured_source","name":"front_camera/","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto heartbeat = stream_control_payloads::parseSubscriptionHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 4U);
  EXPECT_EQ(heartbeat.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(heartbeat.subscriptions[0].target.name, "/camera/front");
  EXPECT_EQ(heartbeat.subscriptions[1].target.kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(heartbeat.subscriptions[1].target.name, "/camera/front");
  EXPECT_EQ(heartbeat.subscriptions[2].target.name, "front_camera");
  EXPECT_EQ(heartbeat.subscriptions[3].target.name, "front_camera/");
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusSerializesVideoDeliveriesAndOptionalFields)
{
  auto topic_video = makeStatus(
    SubscriptionTargetKind::Topic, "/camera/image", SubscriptionDeliveryKind::kVideo, "ros.video.camera.image");
  topic_video.interface_type = "sensor_msgs/msg/Image";
  topic_video.degraded_reason = "source warming up";

  auto configured_source_video = makeStatus(
    SubscriptionTargetKind::ConfiguredSource,
    "/sources/front",
    SubscriptionDeliveryKind::kVideo,
    "ros.video.configured_source.%2Fsources%2Ffront");

  EXPECT_EQ(
    stream_control_payloads::serializeSubscriptionStatus(topic_video),
    nlohmann::json(
      {{"kind", "topic"},
       {"name", "/camera/image"},
       {"status", "active"},
       {"degraded_reason", "source warming up"},
       {"interface_type", "sensor_msgs/msg/Image"},
       {"delivery", {{"kind", "video"}, {"track_name", "ros.video.camera.image"}}}}));

  EXPECT_EQ(
    stream_control_payloads::serializeSubscriptionStatus(configured_source_video),
    nlohmann::json(
      {{"kind", "configured_source"},
       {"name", "/sources/front"},
       {"status", "active"},
       {"delivery", {{"kind", "video"}, {"track_name", "ros.video.configured_source.%2Fsources%2Ffront"}}}}));
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusSerializesDataDeliveries)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/lidar/points", SubscriptionDeliveryKind::kData, "ros.data.lidar.points");
  topic_data.interface_type = "sensor_msgs/msg/PointCloud2";
  topic_data.applied_interval_ms = 0;

  EXPECT_EQ(
    stream_control_payloads::serializeSubscriptionStatus(topic_data),
    nlohmann::json(
      {{"kind", "topic"},
       {"name", "/lidar/points"},
       {"status", "active"},
       {"interface_type", "sensor_msgs/msg/PointCloud2"},
       {"delivery",
        {{"kind", "data"},
         {"track_name", "ros.data.lidar.points"},
         {"content_type", "application/x-ros-cdr"},
         {"interval_ms", 0}}}}));
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusRejectsUnknownDeliveryKind)
{
  SubscriptionStatus status;
  status.target = {SubscriptionTargetKind::Topic, "/camera/image"};
  status.delivery_kind = static_cast<SubscriptionDeliveryKind>(99);

  EXPECT_THROW((void)stream_control_payloads::serializeSubscriptionStatus(status), std::invalid_argument);
}

}  // namespace
}  // namespace livekit_ros2_bridge
