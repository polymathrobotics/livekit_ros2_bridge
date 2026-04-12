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

void expectSingleDemand(
  const nlohmann::json & body,
  SubscriptionTargetKind expected_kind,
  const std::string & expected_name,
  std::optional<int> expected_interval_ms)
{
  const auto heartbeat = parseSubscriptionHeartbeat(body);
  ASSERT_EQ(heartbeat.subscriptions.size(), 1U);

  const SubscriptionDemand & demand = heartbeat.subscriptions[0];
  EXPECT_EQ(demand.target.kind, expected_kind);
  EXPECT_EQ(demand.target.name, expected_name);
  EXPECT_EQ(demand.preferred_interval_ms, expected_interval_ms);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatExtractsNormalizedTopicAndInterval)
{
  expectSingleDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":" battery ","delivery_preferences":{"interval_ms":125},"accepts":"application/x-ros-cdr"}]})"),
    SubscriptionTargetKind::Topic,
    "/battery",
    125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatExtractsConfiguredSourceAndInterval)
{
  expectSingleDemand(
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"configured_source","name":" front_camera ","delivery_preferences":{"interval_ms":125}}]})"),
    SubscriptionTargetKind::ConfiguredSource,
    "front_camera",
    125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatDefaultsIntervalToNullopt)
{
  expectSingleDemand(
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":"/camera"}]})"),
    SubscriptionTargetKind::Topic,
    "/camera",
    std::nullopt);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsMissingOrMalformedSubscriptions)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json::parse(R"({})"),
    nlohmann::json::parse(R"({"subscriptions":{"topic":"/battery"}})"),
    nlohmann::json::parse(R"({"subscriptions":["/battery",{"topic":"/ok"}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsInvalidTargets)
{
  const std::array<nlohmann::json, 5> payloads{
    nlohmann::json::parse(R"({"subscriptions":[{"delivery_preferences":{"interval_ms":100}}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic"}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"name":"/battery"}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"configured_source","name":""}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"topic","name":123}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsUnknownKindsAndLegacyTargetFields)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json::parse(R"({"subscriptions":[{"kind":"service","name":"/battery"}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/battery"}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"configured_source":"/sources/x"}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsInvalidIntervalTypes)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":{"interval_ms":"125"}}]})"),
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":{"interval_ms":125.5}}]})"),
    nlohmann::json::parse(
      R"({"subscriptions":[{"kind":"topic","name":"/lidar","delivery_preferences":{"interval_ms":null}}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatClampsOversizedIntervals)
{
  const auto oversized_interval_ms = std::numeric_limits<std::int64_t>::max();
  const auto expected_clamped_interval_ms = std::numeric_limits<int>::max();
  const nlohmann::json body = {
    {"subscriptions",
     {{{"kind", "topic"}, {"name", "/lidar"}, {"delivery_preferences", {{"interval_ms", oversized_interval_ms}}}}}}};
  expectSingleDemand(body, SubscriptionTargetKind::Topic, "/lidar", expected_clamped_interval_ms);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatCoalescesDuplicateTopicsUsingMinimumInterval)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":25}},
      {"kind":"topic","name":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto update = parseSubscriptionHeartbeat(body);

  ASSERT_EQ(update.subscriptions.size(), 1U);
  EXPECT_EQ(update.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(update.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(update.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatKeepsTopicAndConfiguredSourceDistinctWhenNamesMatch)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"topic","name":"/camera/front","delivery_preferences":{"interval_ms":25}},
      {"kind":"configured_source","name":"/camera/front","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto update = parseSubscriptionHeartbeat(body);

  ASSERT_EQ(update.subscriptions.size(), 2U);
  EXPECT_EQ(update.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(update.subscriptions[0].target.name, "/camera/front");
  EXPECT_EQ(update.subscriptions[0].preferred_interval_ms, 25);
  EXPECT_EQ(update.subscriptions[1].target.kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(update.subscriptions[1].target.name, "/camera/front");
  EXPECT_EQ(update.subscriptions[1].preferred_interval_ms, 125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatKeepsConfiguredSourceSlashVariantsDistinct)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"kind":"configured_source","name":"front_camera","delivery_preferences":{"interval_ms":25}},
      {"kind":"configured_source","name":"front_camera/","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto update = parseSubscriptionHeartbeat(body);

  ASSERT_EQ(update.subscriptions.size(), 2U);
  EXPECT_EQ(update.subscriptions[0].target.name, "front_camera");
  EXPECT_EQ(update.subscriptions[1].target.name, "front_camera/");
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForVideoDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::Topic, "/camera/image"};
  stream_status.interface_type = "sensor_msgs/msg/Image";
  stream_status.delivery_kind = StreamDeliveryKind::kVideo;
  stream_status.track_name = "ros.video.camera.image";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "topic"},
      {"name", "/camera/image"},
      {"status", "active"},
      {"interface_type", "sensor_msgs/msg/Image"},
      {"delivery",
       {
         {"kind", "video"},
         {"track_name", "ros.video.camera.image"},
       }},
    }));
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForConfiguredSourceDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::ConfiguredSource, "/sources/front"};
  stream_status.delivery_kind = StreamDeliveryKind::kVideo;
  stream_status.track_name = "ros.video.configured_source.%2Fsources%2Ffront";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "configured_source"},
      {"name", "/sources/front"},
      {"status", "active"},
      {"delivery",
       {
         {"kind", "video"},
         {"track_name", "ros.video.configured_source.%2Fsources%2Ffront"},
       }},
    }));
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForDataTrackDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::Topic, "/lidar/points"};
  stream_status.interface_type = "sensor_msgs/msg/PointCloud2";
  stream_status.applied_interval_ms = 0;
  stream_status.delivery_kind = StreamDeliveryKind::kData;
  stream_status.track_name = "ros.data.lidar.points";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "topic"},
      {"name", "/lidar/points"},
      {"status", "active"},
      {"interface_type", "sensor_msgs/msg/PointCloud2"},
      {"delivery",
       {
         {"kind", "data"},
         {"track_name", "ros.data.lidar.points"},
         {"content_type", "application/x-ros-cdr"},
         {"interval_ms", 0},
       }},
    }));
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusRejectsUnknownDeliveryKind)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::Topic, "/camera/image"};
  stream_status.delivery_kind = static_cast<StreamDeliveryKind>(99);

  EXPECT_THROW((void)serializeStreamStatus(stream_status), std::invalid_argument);
}

}  // namespace
}  // namespace livekit_ros2_bridge
