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
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "wire/protocol.hpp"
#include "wire/subscriptions.hpp"

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
  const auto heartbeat = wire::subscriptions::parseHeartbeat(body);
  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);

  const SubscriptionDemand & demand = heartbeat.subscriptions[0];
  EXPECT_EQ(demand.kind, expected_kind);
  EXPECT_EQ(demand.name, expected_name);
  EXPECT_EQ(demand.preferred_interval_ms, expected_interval_ms);
}

void expectParseError(const nlohmann::json & body)
{
  EXPECT_THROW((void)wire::subscriptions::parseHeartbeat(body), std::invalid_argument);
}

SubscriptionStatus makeStatus(
  SubscriptionTargetKind target_kind,
  std::string target_name,
  SubscriptionDeliveryKind delivery_kind,
  std::string track_name)
{
  SubscriptionStatus status;
  status.kind = target_kind;
  status.name = std::move(target_name);
  status.delivery_kind = delivery_kind;
  status.track_name = std::move(track_name);
  return status;
}

SubscriptionErrorStatus makeErrorStatus(
  SubscriptionTargetKind target_kind,
  std::string target_name,
  SubscriptionStatusErrorReason reason,
  std::string message)
{
  return {target_kind, std::move(target_name), reason, std::move(message)};
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
  const auto trimmed = wire::subscriptions::parseHeartbeat(nlohmann::json::parse(R"({
      "session_id":"  session-1  ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })"));
  ASSERT_TRUE(trimmed.session_id.has_value());
  EXPECT_EQ(*trimmed.session_id, "session-1");

  const auto missing = wire::subscriptions::parseHeartbeat(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/battery"}]})"));
  EXPECT_EQ(missing.session_id, std::nullopt);

  const auto blank = wire::subscriptions::parseHeartbeat(nlohmann::json::parse(R"({
      "session_id":"   ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })"));
  EXPECT_EQ(blank.session_id, std::nullopt);

  const auto null_id = wire::subscriptions::parseHeartbeat(nlohmann::json::parse(R"({
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
  const auto heartbeat = wire::subscriptions::parseHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);
  EXPECT_EQ(heartbeat.subscriptions[0].name, "/battery");
  EXPECT_EQ(heartbeat.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatCoalescesDuplicateConfiguredSourcesUsingTrimmedName)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"configured_source","name":" front_camera ","delivery_preferences":{"interval_ms":125}},
      {"kind":" configured_source ","name":"front_camera","delivery_preferences":{"interval_ms":25}}
    ]})");
  const auto heartbeat = wire::subscriptions::parseHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);
  EXPECT_EQ(heartbeat.subscriptions[0].kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(heartbeat.subscriptions[0].name, "front_camera");
  EXPECT_EQ(heartbeat.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatTreatsZeroIntervalAsNoPreferenceDuringCoalescing)
{
  const auto zero_then_non_zero = wire::subscriptions::parseHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":0}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})"));
  ASSERT_EQ(zero_then_non_zero.subscriptions.size(), 1U);
  EXPECT_EQ(zero_then_non_zero.subscriptions[0].name, "/battery");
  EXPECT_EQ(zero_then_non_zero.subscriptions[0].preferred_interval_ms, 125);

  const auto non_zero_then_zero = wire::subscriptions::parseHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":0}}
    ]})"));
  ASSERT_EQ(non_zero_then_zero.subscriptions.size(), 1U);
  EXPECT_EQ(non_zero_then_zero.subscriptions[0].name, "/battery");
  EXPECT_EQ(non_zero_then_zero.subscriptions[0].preferred_interval_ms, 125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatTreatsEmptyDeliveryPreferencesAsNoPreferenceDuringCoalescing)
{
  const auto empty_then_non_zero = wire::subscriptions::parseHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})"));
  ASSERT_EQ(empty_then_non_zero.subscriptions.size(), 1U);
  EXPECT_EQ(empty_then_non_zero.subscriptions[0].name, "/battery");
  EXPECT_EQ(empty_then_non_zero.subscriptions[0].preferred_interval_ms, 125);

  const auto non_zero_then_empty = wire::subscriptions::parseHeartbeat(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{}}
    ]})"));
  ASSERT_EQ(non_zero_then_empty.subscriptions.size(), 1U);
  EXPECT_EQ(non_zero_then_empty.subscriptions[0].name, "/battery");
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
  const auto heartbeat = wire::subscriptions::parseHeartbeat(body);

  ASSERT_EQ(heartbeat.subscriptions.size(), 4U);
  EXPECT_EQ(heartbeat.subscriptions[0].kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(heartbeat.subscriptions[0].name, "/camera/front");
  EXPECT_EQ(heartbeat.subscriptions[1].kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(heartbeat.subscriptions[1].name, "/camera/front");
  EXPECT_EQ(heartbeat.subscriptions[2].name, "front_camera");
  EXPECT_EQ(heartbeat.subscriptions[3].name, "front_camera/");
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesSerializesSuccessOnlyBody)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/lidar/points", SubscriptionDeliveryKind::kData, "ros.data.lidar.points");
  topic_data.interface_type = "sensor_msgs/msg/PointCloud2";
  topic_data.applied_interval_ms = 50;

  auto configured_source_video = makeStatus(
    SubscriptionTargetKind::ConfiguredSource,
    "/sources/front",
    SubscriptionDeliveryKind::kVideo,
    "ros.video.configured_source.%2Fsources%2Ffront");
  configured_source_video.degraded_reason = "source warming up";

  nlohmann::json expected = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kControlSubscriptionsStatus},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/lidar/points"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/PointCloud2"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "ros.data.lidar.points"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 50}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "configured_source"},
    {"name", "/sources/front"},
    {"status", "active"},
    {"degraded_reason", "source warming up"},
    {"delivery", {{"kind", "video"}, {"track_name", "ros.video.configured_source.%2Fsources%2Ffront"}}},
  });

  EXPECT_EQ(
    wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{topic_data, configured_source_video},
      std::nullopt,
      std::nullopt,
      std::chrono::steady_clock::time_point{}),
    expected);
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesSerializesErrorOnlyBody)
{
  nlohmann::json expected = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kControlSubscriptionsStatus},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "error"},
    {"error", {{"reason", "forbidden"}, {"message", "ROS topic '/battery_state' not permitted."}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/camera/front"},
    {"status", "error"},
    {"error", {{"reason", "unavailable"}, {"message", "Video stream registry is unavailable."}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "configured_source"},
    {"name", "/sources/missing"},
    {"status", "error"},
    {"error", {{"reason", "not_found"}, {"message", "Unknown configured video source '/sources/missing'."}}},
  });

  EXPECT_EQ(
    wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{
        makeErrorStatus(
          SubscriptionTargetKind::Topic,
          "/battery_state",
          SubscriptionStatusErrorReason::kForbidden,
          "ROS topic '/battery_state' not permitted."),
        makeErrorStatus(
          SubscriptionTargetKind::Topic,
          "/camera/front",
          SubscriptionStatusErrorReason::kUnavailable,
          "Video stream registry is unavailable."),
        makeErrorStatus(
          SubscriptionTargetKind::ConfiguredSource,
          "/sources/missing",
          SubscriptionStatusErrorReason::kNotFound,
          "Unknown configured video source '/sources/missing'."),
      },
      std::nullopt,
      std::nullopt,
      std::chrono::steady_clock::time_point{}),
    expected);
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesSerializesSessionAndExpiryMetadata)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/battery_state", SubscriptionDeliveryKind::kData, "ros.data.battery_state");
  topic_data.interface_type = "sensor_msgs/msg/BatteryState";
  topic_data.applied_interval_ms = 100;

  const auto now = std::chrono::steady_clock::time_point{std::chrono::milliseconds(1000)};
  const auto session_id = std::optional<std::string>{"session-1"};
  const auto expiry = std::optional<std::chrono::steady_clock::time_point>{now + std::chrono::milliseconds(45000)};

  nlohmann::json expected = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kControlSubscriptionsStatus},
    {"session_id", "session-1"},
    {"lease_expires_in_ms", 45000},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "ros.data.battery_state"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 100}}},
  });

  EXPECT_EQ(
    wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{topic_data}, session_id, expiry, now),
    expected);
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesSerializesExpiryWithoutSessionId)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/battery_state", SubscriptionDeliveryKind::kData, "ros.data.battery_state");
  topic_data.interface_type = "sensor_msgs/msg/BatteryState";
  topic_data.applied_interval_ms = 100;

  const auto now = std::chrono::steady_clock::time_point{std::chrono::milliseconds(1000)};
  const auto expiry = std::optional<std::chrono::steady_clock::time_point>{now + std::chrono::milliseconds(45000)};

  nlohmann::json expected = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kControlSubscriptionsStatus},
    {"lease_expires_in_ms", 45000},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "ros.data.battery_state"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 100}}},
  });

  EXPECT_EQ(
    wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{topic_data}, std::nullopt, expiry, now),
    expected);
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesSerializesMixedStatuses)
{
  auto configured_source_video = makeStatus(
    SubscriptionTargetKind::ConfiguredSource,
    "/sources/front",
    SubscriptionDeliveryKind::kVideo,
    "ros.video.configured_source.%2Fsources%2Ffront");

  nlohmann::json expected = {
    {"v", wire::protocol::kProtocolVersion},
    {"type", wire::protocol::kControlSubscriptionsStatus},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "configured_source"},
    {"name", "/sources/front"},
    {"status", "active"},
    {"delivery", {{"kind", "video"}, {"track_name", "ros.video.configured_source.%2Fsources%2Ffront"}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/nonexistent_topic"},
    {"status", "error"},
    {"error", {{"reason", "not_found"}, {"message", "No ROS types found for topic '/nonexistent_topic'."}}},
  });

  EXPECT_EQ(
    wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{
        configured_source_video,
        makeErrorStatus(
          SubscriptionTargetKind::Topic,
          "/nonexistent_topic",
          SubscriptionStatusErrorReason::kNotFound,
          "No ROS types found for topic '/nonexistent_topic'."),
      },
      std::nullopt,
      std::nullopt,
      std::chrono::steady_clock::time_point{}),
    expected);
}

TEST(StreamControlPayloadsTest, SerializeSubscriptionStatusesRejectsUnknownDeliveryKind)
{
  SubscriptionStatus status;
  status.kind = SubscriptionTargetKind::Topic;
  status.name = "/camera/image";
  status.delivery_kind = static_cast<SubscriptionDeliveryKind>(99);

  EXPECT_THROW(
    (void)wire::subscriptions::serializeStatuses(
      std::vector<SubscriptionReportedStatus>{status},
      std::nullopt,
      std::nullopt,
      std::chrono::steady_clock::time_point{}),
    std::invalid_argument);
}

}  // namespace
}  // namespace livekit_ros2_bridge
