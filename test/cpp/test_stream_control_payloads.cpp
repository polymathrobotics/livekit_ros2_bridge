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
#include "utils/subscription_target_naming.hpp"

namespace livekit_ros2_bridge
{
namespace
{

void expectSingleSubscription(
  const nlohmann::json & body,
  SubscriptionTargetKind expected_kind,
  const std::string & expected_name,
  std::optional<int> expected_interval_ms)
{
  const auto update = parseSubscriptionHeartbeat(body);
  ASSERT_EQ(update.subscriptions.size(), 1U);

  const SubscriptionRequest & entry = update.subscriptions[0];
  EXPECT_EQ(entry.target.kind, expected_kind);
  EXPECT_EQ(entry.target.name, expected_name);
  EXPECT_EQ(entry.preferred_interval_ms, expected_interval_ms);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatExtractsNormalizedTopicAndInterval)
{
  expectSingleSubscription(
    nlohmann::json::parse(
      R"({"subscriptions":[{"topic":" battery ","delivery_preferences":{"interval_ms":125},"accepts":"application/x-ros-cdr"}]})"),
    SubscriptionTargetKind::Topic,
    "/battery",
    125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatExtractsExternalAndInterval)
{
  expectSingleSubscription(
    nlohmann::json::parse(
      R"({"subscriptions":[{"external":"/cameras/front","delivery_preferences":{"interval_ms":125}}]})"),
    SubscriptionTargetKind::External,
    "/cameras/front",
    125);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatDefaultsIntervalToNullopt)
{
  expectSingleSubscription(
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/camera"}]})"),
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
  const std::array<nlohmann::json, 4> payloads{
    nlohmann::json::parse(R"({"subscriptions":[{"delivery_preferences":{"interval_ms":100}}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"external":""}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"topic":123}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/battery","external":"/sources/x"}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatRejectsInvalidIntervalTypes)
{
  const std::array<nlohmann::json, 3> payloads{
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/lidar","delivery_preferences":{"interval_ms":"125"}}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/lidar","delivery_preferences":{"interval_ms":125.5}}]})"),
    nlohmann::json::parse(R"({"subscriptions":[{"topic":"/lidar","delivery_preferences":{"interval_ms":null}}]})"),
  };

  for (const auto & body : payloads) {
    EXPECT_THROW((void)parseSubscriptionHeartbeat(body), std::invalid_argument);
  }
}

TEST(StreamControlPayloadsTest, ParseHeartbeatClampsOversizedIntervals)
{
  const nlohmann::json body = {
    {"subscriptions",
     {{{"topic", "/lidar"}, {"delivery_preferences", {{"interval_ms", std::numeric_limits<std::int64_t>::max()}}}}}}};
  expectSingleSubscription(body, SubscriptionTargetKind::Topic, "/lidar", std::numeric_limits<int>::max());
}

TEST(StreamControlPayloadsTest, ParseHeartbeatCoalescesDuplicateTopicsUsingMinimumInterval)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"topic":"/battery","delivery_preferences":{"interval_ms":25}},
      {"topic":" /battery ","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto update = parseSubscriptionHeartbeat(body);

  ASSERT_EQ(update.subscriptions.size(), 1U);
  EXPECT_EQ(update.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(update.subscriptions[0].target.name, "/battery");
  EXPECT_EQ(update.subscriptions[0].preferred_interval_ms, 25);
}

TEST(StreamControlPayloadsTest, ParseHeartbeatKeepsTopicAndExternalDistinctWhenNamesMatch)
{
  const auto body = nlohmann::json::parse(
    R"({"subscriptions":[
      {"topic":"/camera/front","delivery_preferences":{"interval_ms":25}},
      {"external":"/camera/front","delivery_preferences":{"interval_ms":125}}
    ]})");
  const auto update = parseSubscriptionHeartbeat(body);

  ASSERT_EQ(update.subscriptions.size(), 2U);
  EXPECT_EQ(update.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(update.subscriptions[0].target.name, "/camera/front");
  EXPECT_EQ(update.subscriptions[0].preferred_interval_ms, 25);
  EXPECT_EQ(update.subscriptions[1].target.kind, SubscriptionTargetKind::External);
  EXPECT_EQ(update.subscriptions[1].target.name, "/camera/front");
  EXPECT_EQ(update.subscriptions[1].preferred_interval_ms, 125);
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForVideoDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::Topic, "/camera/image"};
  stream_status.interface_type = "sensor_msgs/msg/Image";
  stream_status.delivery_kind = "video";
  stream_status.publisher_identity = "bridge-video-camera-image";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "topic"},
      {"topic", "/camera/image"},
      {"status", "active"},
      {"interface_type", "sensor_msgs/msg/Image"},
      {"delivery",
       {
         {"kind", "video"},
         {"publisher_identity", "bridge-video-camera-image"},
         {"track_name", ""},
       }},
    }));
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForConfiguredSourceDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::External, "/sources/front"};
  stream_status.delivery_kind = "video";
  stream_status.publisher_identity = "bridge-video-source-sources-front";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "external"},
      {"external", "/sources/front"},
      {"status", "active"},
      {"delivery",
       {
         {"kind", "video"},
         {"publisher_identity", "bridge-video-source-sources-front"},
         {"track_name", ""},
       }},
    }));
}

TEST(StreamControlPayloadsTest, SerializeStreamStatusForDataTrackDelivery)
{
  StreamStatus stream_status;
  stream_status.target = {SubscriptionTargetKind::Topic, "/lidar/points"};
  stream_status.interface_type = "sensor_msgs/msg/PointCloud2";
  stream_status.applied_interval_ms = 0;
  stream_status.delivery_kind = "data_track";
  stream_status.track_name = "ros.cdr.lidar.points";

  const auto entry = serializeStreamStatus(stream_status);

  EXPECT_EQ(
    entry,
    (nlohmann::json{
      {"kind", "topic"},
      {"topic", "/lidar/points"},
      {"status", "active"},
      {"interface_type", "sensor_msgs/msg/PointCloud2"},
      {"delivery",
       {
         {"kind", "data_track"},
         {"track_name", "ros.cdr.lidar.points"},
         {"content_type", "application/x-ros-cdr"},
       }},
      {"applied_preferences", {{"interval_ms", 0}}},
    }));
}

TEST(StreamControlPayloadsTest, DeriveVideoTrackNameUsesExpectedPrefixes)
{
  struct TestCase
  {
    SubscriptionTargetKind kind;
    const char * name;
    const char * expected_track_name;
  };

  const std::array<TestCase, 4> test_cases{{
    {SubscriptionTargetKind::Topic, "/camera/front/image", "ros.video.camera.front.image"},
    {SubscriptionTargetKind::Topic, "/battery", "ros.video.battery"},
    {SubscriptionTargetKind::External, "/sources/dev/x", "ros.video.external.sources.dev.x"},
    {SubscriptionTargetKind::External, "/sources/front", "ros.video.external.sources.front"},
  }};

  for (const auto & test_case : test_cases) {
    EXPECT_EQ(deriveVideoTrackName(test_case.kind, test_case.name), test_case.expected_track_name);
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge
