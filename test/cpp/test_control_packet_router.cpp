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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control_packet_router.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"

namespace livekit_ros2_bridge
{
namespace
{

rclcpp::Clock::SharedPtr makeTestClock()
{
  return std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}

std::vector<std::uint8_t> toBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

std::string heartbeatPayload()
{
  return R"({"session_id":"session-1","subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}}]})";
}

std::string publishPayload()
{
  return nlohmann::json{
    {"topic", "/battery/cmd"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"message", cdr_payload::serialize(std::vector<std::uint8_t>{0x01U, 0x02U})},
  }
    .dump();
}

IncomingControlPacket makePacket(
  const std::string & payload, const std::string & topic, std::string requester_identity = "participant-1")
{
  return IncomingControlPacket{toBytes(payload), topic, std::move(requester_identity)};
}

struct HeartbeatCall final
{
  std::string requester_identity;
  SubscriptionHeartbeat heartbeat;
};

struct PublishCall final
{
  std::string requester_identity;
  TopicPublishCommand command;
};

struct RoutingProbe final
{
  std::optional<HeartbeatCall> heartbeat;
  std::optional<PublishCall> publish;

  ControlPacketRouter makeRouter()
  {
    auto clock = makeTestClock();
    ControlPacketRouter::Handlers handlers;
    handlers.heartbeat_handler = [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
      this->heartbeat = HeartbeatCall{std::move(requester_identity), std::move(heartbeat)};
    };
    handlers.publish_handler = [this](std::string requester_identity, TopicPublishCommand command) {
      this->publish = PublishCall{std::move(requester_identity), std::move(command)};
    };
    return ControlPacketRouter(rclcpp::get_logger("control_packet_router_test"), std::move(clock), std::move(handlers));
  }
};

RoutingProbe routePacket(const IncomingControlPacket & packet)
{
  RoutingProbe probe;
  auto router = probe.makeRouter();
  router.route(packet);
  return probe;
}

void expectNoDispatch(const IncomingControlPacket & packet)
{
  const auto probe = routePacket(packet);
  EXPECT_FALSE(probe.heartbeat.has_value());
  EXPECT_FALSE(probe.publish.has_value());
}

}  // namespace

TEST(ControlPacketRouterTest, RoutesHeartbeatPayloadsToTheHeartbeatHandler)
{
  const auto expect_route = [](std::string requester_identity) {
    const auto probe =
      routePacket(makePacket(heartbeatPayload(), protocol::kControlSubscriptionsHeartbeat, requester_identity));

    ASSERT_TRUE(probe.heartbeat.has_value());
    EXPECT_FALSE(probe.publish.has_value());
    EXPECT_EQ(probe.heartbeat->requester_identity, requester_identity);
    ASSERT_TRUE(probe.heartbeat->heartbeat.session_id.has_value());
    EXPECT_EQ(*probe.heartbeat->heartbeat.session_id, "session-1");
    ASSERT_EQ(probe.heartbeat->heartbeat.subscriptions.size(), 1U);
    const auto & subscription = probe.heartbeat->heartbeat.subscriptions.front();
    EXPECT_EQ(subscription.target.kind, SubscriptionTargetKind::Topic);
    EXPECT_EQ(subscription.target.name, "/battery");
    ASSERT_TRUE(subscription.preferred_interval_ms.has_value());
    EXPECT_EQ(*subscription.preferred_interval_ms, 125);
  };

  // Heartbeats intentionally cover both the direct requester case and the
  // anonymous/session-fallback path.
  expect_route("participant-1");
  expect_route("");
}

TEST(ControlPacketRouterTest, RoutesPublishPayloads)
{
  const auto probe = routePacket(makePacket(publishPayload(), protocol::kControlTopicPublish));

  EXPECT_FALSE(probe.heartbeat.has_value());
  ASSERT_TRUE(probe.publish.has_value());
  EXPECT_EQ(probe.publish->requester_identity, "participant-1");
  EXPECT_EQ(probe.publish->command.topic, "/battery/cmd");
  EXPECT_EQ(probe.publish->command.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(probe.publish->command.cdr, (std::vector<std::uint8_t>{0x01U, 0x02U}));
}

TEST(ControlPacketRouterTest, ContainsHeartbeatHandlerExceptionsAndContinuesRouting)
{
  auto clock = makeTestClock();
  int heartbeat_call_count = 0;
  bool publish_called = false;
  std::optional<SubscriptionHeartbeat> next_heartbeat;

  ControlPacketRouter router(
    rclcpp::get_logger("control_packet_router_test"),
    std::move(clock),
    ControlPacketRouter::Handlers{
      [&](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
        ++heartbeat_call_count;
        if (heartbeat_call_count == 1) {
          throw std::runtime_error("heartbeat handler failed");
        }
        EXPECT_EQ(requester_identity, "participant-1");
        next_heartbeat = std::move(heartbeat);
      },
      [&](std::string, TopicPublishCommand) { publish_called = true; },
    });

  const auto packet = makePacket(heartbeatPayload(), protocol::kControlSubscriptionsHeartbeat);
  EXPECT_NO_THROW(router.route(packet));
  EXPECT_NO_THROW(router.route(packet));

  EXPECT_EQ(heartbeat_call_count, 2);
  EXPECT_FALSE(publish_called);
  ASSERT_TRUE(next_heartbeat.has_value());
}

TEST(ControlPacketRouterTest, ContainsPublishHandlerExceptionsAndContinuesRouting)
{
  auto clock = makeTestClock();
  int publish_call_count = 0;
  bool heartbeat_called = false;
  std::optional<TopicPublishCommand> next_command;

  ControlPacketRouter router(
    rclcpp::get_logger("control_packet_router_test"),
    std::move(clock),
    ControlPacketRouter::Handlers{
      [&](std::string, SubscriptionHeartbeat) { heartbeat_called = true; },
      [&](std::string requester_identity, TopicPublishCommand command) {
        ++publish_call_count;
        if (publish_call_count == 1) {
          throw std::runtime_error("publish handler failed");
        }
        EXPECT_EQ(requester_identity, "participant-1");
        next_command = std::move(command);
      },
    });

  const auto packet = makePacket(publishPayload(), protocol::kControlTopicPublish);
  EXPECT_NO_THROW(router.route(packet));
  EXPECT_NO_THROW(router.route(packet));

  EXPECT_EQ(publish_call_count, 2);
  EXPECT_FALSE(heartbeat_called);
  ASSERT_TRUE(next_command.has_value());
}

TEST(ControlPacketRouterTest, RejectsMalformedHeartbeatPayloadsWithoutDispatch)
{
  expectNoDispatch(makePacket("{", protocol::kControlSubscriptionsHeartbeat));
}

TEST(ControlPacketRouterTest, RejectsStructurallyInvalidHeartbeatPayloadsWithoutDispatch)
{
  expectNoDispatch(makePacket(R"({})", protocol::kControlSubscriptionsHeartbeat));
}

TEST(ControlPacketRouterTest, RejectsAnonymousPublishPacketsWithoutDispatch)
{
  expectNoDispatch(makePacket(publishPayload(), protocol::kControlTopicPublish, ""));
}

TEST(ControlPacketRouterTest, RejectsInvalidPublishPayloadsWithoutDispatch)
{
  expectNoDispatch(makePacket("{", protocol::kControlTopicPublish));
}

TEST(ControlPacketRouterTest, DropsUnsupportedControlTopicsWithoutDispatch)
{
  expectNoDispatch(IncomingControlPacket{{'x'}, "custom.topic", "participant-1"});
}

TEST(ControlPacketRouterTest, ValidatesConstructorDependencies)
{
  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      {},
      ControlPacketRouter::Handlers{
        [](std::string, SubscriptionHeartbeat) {},
        [](std::string, TopicPublishCommand) {},
      }),
    std::invalid_argument);
  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      makeTestClock(),
      ControlPacketRouter::Handlers{
        {},
        [](std::string, TopicPublishCommand) {},
      }),
    std::invalid_argument);

  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      makeTestClock(),
      ControlPacketRouter::Handlers{
        [](std::string, SubscriptionHeartbeat) {},
        {},
      }),
    std::invalid_argument);
}

}  // namespace livekit_ros2_bridge
