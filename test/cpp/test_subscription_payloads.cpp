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
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/constants.hpp"
#include "protocol/subscriptions_json.hpp"
#include "protocol/validation_error.hpp"
#include "protocol_test_support.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

namespace livekit_ros2_bridge
{
namespace
{

constexpr char kTestTopicExpansionNodeName[] = "test_subscription_payloads";
constexpr char kTestTopicExpansionNamespace[] = "/";
using test_support::expectInvalidArgument;

std::vector<std::uint8_t> payloadBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

std::string expandHeartbeatTopicName(const std::string & topic)
{
  return rclcpp::expand_topic_or_service_name(topic, kTestTopicExpansionNodeName, kTestTopicExpansionNamespace);
}

SubscriptionHeartbeat parsePayload(const std::string & payload)
{
  return protocol::subscriptions::parse(payloadBytes(payload));
}

void expectDemand(
  const nlohmann::json & body,
  SubscriptionTargetKind expected_kind,
  const std::string & expected_name,
  std::optional<int> expected_interval_ms)
{
  const auto heartbeat = parsePayload(body.dump());
  ASSERT_EQ(heartbeat.demands.size(), 1U);

  const SubscriptionDemand & demand = heartbeat.demands[0];
  EXPECT_EQ(demand.kind, expected_kind);
  EXPECT_EQ(demand.name, expected_name);
  EXPECT_EQ(demand.preferred_interval_ms, expected_interval_ms);
}

void expectParseError(const nlohmann::json & body, const char * expected_message, const char * expected_field)
{
  expectInvalidArgument([&body]() { (void)parsePayload(body.dump()); }, expected_message, expected_field);
}

SubscriptionStatus makeStatus(
  SubscriptionTargetKind kind, std::string name, SubscriptionDeliveryKind delivery, std::string track_name)
{
  SubscriptionStatus status;
  status.kind = kind;
  status.name = std::move(name);
  status.delivery = delivery;
  status.track_name = std::move(track_name);
  return status;
}

SubscriptionErrorStatus makeErrorStatus(
  SubscriptionTargetKind kind, std::string name, SubscriptionErrorReason reason, std::string message)
{
  return {kind, std::move(name), reason, std::move(message)};
}

nlohmann::json statusBody(
  const std::vector<SubscriptionStatusEntry> & statuses,
  const std::optional<std::string> & session_id,
  const std::optional<std::chrono::steady_clock::time_point> & expiry)
{
  const SubscriptionStatusReport report{statuses, session_id, expiry};
  return nlohmann::json::parse(protocol::subscriptions::serialize(report));
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatNormalizesTargetsAndIntervals)
{
  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":" topic ","name":" battery ","delivery_preferences":{"interval_ms":125},"accepts":"application/x-ros-cdr"}]})"),
    SubscriptionTargetKind::Topic,
    expandHeartbeatTopicName("battery"),
    125);

  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"other_video","name":" front_camera ","delivery_preferences":{"interval_ms":125}}]})"),
    SubscriptionTargetKind::OtherVideo,
    "front_camera",
    125);

  expectDemand(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/camera"}]})"),
    SubscriptionTargetKind::Topic,
    expandHeartbeatTopicName("/camera"),
    std::nullopt);
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatParsesOptionalSessionIdAndRejectsMistypedValues)
{
  const auto trimmed = parsePayload(R"({
      "session_id":"  session-1  ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })");
  ASSERT_TRUE(trimmed.session_id.has_value());
  EXPECT_EQ(*trimmed.session_id, "session-1");

  const auto missing = parsePayload(R"({"subscriptions":[{"kind":"topic","name":"/battery"}]})");
  EXPECT_EQ(missing.session_id, std::nullopt);

  const auto blank = parsePayload(R"({
      "session_id":"   ",
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })");
  EXPECT_EQ(blank.session_id, std::nullopt);

  const auto null_id = parsePayload(R"({
      "session_id":null,
      "subscriptions":[{"kind":"topic","name":"/battery"}]
    })");
  EXPECT_EQ(null_id.session_id, std::nullopt);

  expectParseError(
    nlohmann::json::parse(R"({
    "session_id":125,
    "subscriptions":[{"kind":"topic","name":"/battery"}]
  })"),
    "heartbeat session_id must be a string",
    "session_id");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatRejectsMissingOrMalformedSubscriptions)
{
  expectParseError(nlohmann::json::parse(R"({})"), "heartbeat subscriptions are required", "subscriptions");
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":{"topic":"/battery"}})"),
    "heartbeat subscriptions must be an array",
    "subscriptions");
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":["/battery"]})"),
    "heartbeat subscriptions must be objects",
    "subscriptions");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatRejectsMalformedPayloads)
{
  expectInvalidArgument([]() { (void)parsePayload("{"); }, "Invalid JSON in subscription heartbeat", "payload");
  expectInvalidArgument(
    []() { (void)parsePayload(R"(["/battery"])"); }, "Subscription heartbeat must be a JSON object", "payload");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatRejectsMissingOrNonStringTargetFields)
{
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"name":"/battery"}]})"),
    "heartbeat subscription 'kind' must be a string",
    "subscriptions.kind");
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic"}]})"),
    "heartbeat subscription 'name' must be a string",
    "subscriptions.name");
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":123}]})"),
    "heartbeat subscription 'name' must be a string",
    "subscriptions.name");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatRejectsBlankOrUnsupportedTargets)
{
  const auto blank_topic_body = nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"   "}]})");
  try {
    (void)parsePayload(blank_topic_body.dump());
    ADD_FAILURE() << "Expected protocol::ValidationError";
  } catch (const std::invalid_argument & error) {
    const auto * validation = dynamic_cast<const protocol::ValidationError *>(&error);
    ASSERT_NE(validation, nullptr);
    EXPECT_EQ(validation->field(), "subscriptions.name");
  }

  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"other_video","name":"   "}]})"),
    "heartbeat subscription other video name must trim to a non-empty name",
    "subscriptions.name");
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"service","name":"/battery"}]})"),
    "heartbeat subscription 'kind' must be 'topic' or 'other_video'",
    "subscriptions.kind");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatRejectsInvalidIntervalTypes)
{
  expectParseError(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":125}]})"),
    "delivery_preferences must be an object",
    "subscriptions.delivery_preferences");
  expectParseError(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":{"interval_ms":"125"}}]})"),
    "delivery_preferences.interval_ms must be an integer",
    "subscriptions.delivery_preferences.interval_ms");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatClampsOutOfRangeIntervals)
{
  const auto expectClampedInterval = [](std::int64_t raw_interval_ms, int expected_interval_ms) {
    const nlohmann::json body = {
      {"subscriptions",
       {{{"kind", "topic"}, {"name", "/lidar"}, {"delivery_preferences", {{"interval_ms", raw_interval_ms}}}}}}};

    expectDemand(body, SubscriptionTargetKind::Topic, expandHeartbeatTopicName("/lidar"), expected_interval_ms);
  };

  expectClampedInterval(std::numeric_limits<std::int64_t>::max(), std::numeric_limits<int>::max());
  expectClampedInterval(std::numeric_limits<std::int64_t>::min(), std::numeric_limits<int>::min());

  const nlohmann::json unsigned_body = {
    {"subscriptions",
     {{{"kind", "topic"},
       {"name", "/lidar"},
       {"delivery_preferences", {{"interval_ms", std::numeric_limits<std::uint64_t>::max()}}}}}}};
  expectDemand(
    unsigned_body, SubscriptionTargetKind::Topic, expandHeartbeatTopicName("/lidar"), std::numeric_limits<int>::max());
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatCoalescesDuplicateTopicsUsingMinimumInterval)
{
  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":25}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})"),
    SubscriptionTargetKind::Topic,
    expandHeartbeatTopicName("/battery"),
    25);
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatCoalescesDuplicateOtherVideoTargetsUsingTrimmedName)
{
  expectDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[
      {"kind":"other_video","name":" front_camera ","delivery_preferences":{"interval_ms":125}},
      {"kind":" other_video ","name":"front_camera","delivery_preferences":{"interval_ms":25}}
    ]})"),
    SubscriptionTargetKind::OtherVideo,
    "front_camera",
    25);
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatTreatsZeroIntervalAsNoPreferenceDuringCoalescing)
{
  const auto expectPreferredInterval = [](const char * body) {
    expectDemand(nlohmann::json::parse(body), SubscriptionTargetKind::Topic, expandHeartbeatTopicName("/battery"), 125);
  };

  expectPreferredInterval(R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":0}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})");
  expectPreferredInterval(R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":0}}
    ]})");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatTreatsEmptyDeliveryPreferencesAsNoPreferenceDuringCoalescing)
{
  const auto expectPreferredInterval = [](const char * body) {
    expectDemand(nlohmann::json::parse(body), SubscriptionTargetKind::Topic, expandHeartbeatTopicName("/battery"), 125);
  };

  expectPreferredInterval(R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})");
  expectPreferredInterval(R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{}}
    ]})");
}

TEST(SubscriptionPayloadsTest, ParseHeartbeatKeepsDistinctTargetsSeparate)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"topic","name":"/camera/front","delivery_preferences":{"interval_ms":25}},
      {"kind":"other_video","name":"/camera/front","delivery_preferences":{"interval_ms":125}},
      {"kind":"other_video","name":"front_camera","delivery_preferences":{"interval_ms":25}},
      {"kind":"other_video","name":"front_camera/","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto heartbeat = parsePayload(body.dump());

  ASSERT_EQ(heartbeat.demands.size(), 4U);
  EXPECT_EQ(heartbeat.demands[0].kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(heartbeat.demands[0].name, expandHeartbeatTopicName("/camera/front"));
  EXPECT_EQ(heartbeat.demands[1].kind, SubscriptionTargetKind::OtherVideo);
  EXPECT_EQ(heartbeat.demands[1].name, "/camera/front");
  EXPECT_EQ(heartbeat.demands[2].name, "front_camera");
  EXPECT_EQ(heartbeat.demands[3].name, "front_camera/");
}

TEST(SubscriptionPayloadsTest, SerializeSubscriptionStatusesSerializesSuccessOnlyBody)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/lidar/points", SubscriptionDeliveryKind::Data, "lkros.data.lidar.points");
  topic_data.interface_type = "sensor_msgs/msg/PointCloud2";
  topic_data.interval_ms = 50;

  auto other_video = makeStatus(
    SubscriptionTargetKind::OtherVideo,
    "/sources/front",
    SubscriptionDeliveryKind::Video,
    "lkros.video.other.%2Fsources%2Ffront");
  other_video.degradation_reason = "source warming up";

  nlohmann::json expected = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/lidar/points"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/PointCloud2"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "lkros.data.lidar.points"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 50}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "other_video"},
    {"name", "/sources/front"},
    {"status", "active"},
    {"degraded_reason", "source warming up"},
    {"delivery", {{"kind", "video"}, {"track_name", "lkros.video.other.%2Fsources%2Ffront"}}},
  });

  EXPECT_EQ(
    statusBody(
      std::vector<SubscriptionStatusEntry>{SubscriptionStatusEntry{topic_data}, SubscriptionStatusEntry{other_video}},
      std::nullopt,
      std::nullopt),
    expected);
}

TEST(SubscriptionPayloadsTest, SerializeSubscriptionStatusesSerializesErrorOnlyBody)
{
  nlohmann::json expected = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "error"},
    {"error", {{"reason", "forbidden"}, {"message", "ROS topic '/battery_state' not permitted."}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "other_video"},
    {"name", "/sources/missing"},
    {"status", "error"},
    {"error", {{"reason", "not_found"}, {"message", "Unknown other video source '/sources/missing'."}}},
  });

  EXPECT_EQ(
    statusBody(
      std::vector<SubscriptionStatusEntry>{
        SubscriptionStatusEntry{makeErrorStatus(
          SubscriptionTargetKind::Topic,
          "/battery_state",
          SubscriptionErrorReason::Forbidden,
          "ROS topic '/battery_state' not permitted.")},
        SubscriptionStatusEntry{makeErrorStatus(
          SubscriptionTargetKind::OtherVideo,
          "/sources/missing",
          SubscriptionErrorReason::NotFound,
          "Unknown other video source '/sources/missing'.")},
      },
      std::nullopt,
      std::nullopt),
    expected);
}

TEST(SubscriptionPayloadsTest, SerializeSubscriptionStatusesSerializesSessionAndExpiryMetadata)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/battery_state", SubscriptionDeliveryKind::Data, "lkros.data.battery_state");
  topic_data.interface_type = "sensor_msgs/msg/BatteryState";
  topic_data.interval_ms = 100;

  const auto session_id = std::optional<std::string>{"session-1"};
  const auto expiry =
    std::optional<std::chrono::steady_clock::time_point>{std::chrono::steady_clock::now() + std::chrono::seconds(45)};

  nlohmann::json expected = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    {"session_id", "session-1"},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "lkros.data.battery_state"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 100}}},
  });

  auto body = statusBody(std::vector<SubscriptionStatusEntry>{SubscriptionStatusEntry{topic_data}}, session_id, expiry);
  ASSERT_TRUE(body["lease_expires_in_ms"].is_number_integer());
  EXPECT_GT(body["lease_expires_in_ms"].get<std::int64_t>(), 0);
  EXPECT_LE(body["lease_expires_in_ms"].get<std::int64_t>(), 45000);
  body.erase("lease_expires_in_ms");
  EXPECT_EQ(body, expected);
}

TEST(SubscriptionPayloadsTest, SerializeSubscriptionStatusesSerializesExpiryWithoutSessionId)
{
  auto topic_data = makeStatus(
    SubscriptionTargetKind::Topic, "/battery_state", SubscriptionDeliveryKind::Data, "lkros.data.battery_state");
  topic_data.interface_type = "sensor_msgs/msg/BatteryState";
  topic_data.interval_ms = 100;

  const auto expiry =
    std::optional<std::chrono::steady_clock::time_point>{std::chrono::steady_clock::now() + std::chrono::seconds(45)};

  nlohmann::json expected = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/battery_state"},
    {"status", "active"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"delivery",
     {{"kind", "data"},
      {"track_name", "lkros.data.battery_state"},
      {"content_type", "application/x-ros-cdr"},
      {"interval_ms", 100}}},
  });

  auto body =
    statusBody(std::vector<SubscriptionStatusEntry>{SubscriptionStatusEntry{topic_data}}, std::nullopt, expiry);
  ASSERT_TRUE(body["lease_expires_in_ms"].is_number_integer());
  EXPECT_GT(body["lease_expires_in_ms"].get<std::int64_t>(), 0);
  EXPECT_LE(body["lease_expires_in_ms"].get<std::int64_t>(), 45000);
  body.erase("lease_expires_in_ms");
  EXPECT_EQ(body, expected);
}

TEST(SubscriptionPayloadsTest, SerializeSubscriptionStatusesSerializesMixedStatuses)
{
  auto other_video = makeStatus(
    SubscriptionTargetKind::OtherVideo,
    "/sources/front",
    SubscriptionDeliveryKind::Video,
    "lkros.video.other.%2Fsources%2Ffront");

  nlohmann::json expected = {
    {"v", protocol::kProtocolVersion},
    {"type", protocol::kStatusTopic},
    {"subscriptions", nlohmann::json::array()},
  };
  expected["subscriptions"].push_back({
    {"kind", "other_video"},
    {"name", "/sources/front"},
    {"status", "active"},
    {"delivery", {{"kind", "video"}, {"track_name", "lkros.video.other.%2Fsources%2Ffront"}}},
  });
  expected["subscriptions"].push_back({
    {"kind", "topic"},
    {"name", "/nonexistent_topic"},
    {"status", "error"},
    {"error", {{"reason", "not_found"}, {"message", "No ROS types found for topic '/nonexistent_topic'."}}},
  });

  EXPECT_EQ(
    statusBody(
      std::vector<SubscriptionStatusEntry>{
        SubscriptionStatusEntry{other_video},
        SubscriptionStatusEntry{makeErrorStatus(
          SubscriptionTargetKind::Topic,
          "/nonexistent_topic",
          SubscriptionErrorReason::NotFound,
          "No ROS types found for topic '/nonexistent_topic'.")},
      },
      std::nullopt,
      std::nullopt),
    expected);
}

}  // namespace
}  // namespace livekit_ros2_bridge
