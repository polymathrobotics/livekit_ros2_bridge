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

std::vector<std::uint8_t> toBytes(const std::string & command_payload)
{
  return std::vector<std::uint8_t>(command_payload.begin(), command_payload.end());
}

std::string makeHeartbeatPayload()
{
  return R"({"session_id":"session-1","subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}}]})";
}

std::string makePublishPayload()
{
  return nlohmann::json{
    {"topic", "/battery/cmd"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"message", serializeCdrPayload(std::vector<std::uint8_t>{0x01U, 0x02U})},
  }
    .dump();
}

IncomingControlPacket makePacket(
  const std::string & command_payload,
  const std::string & control_topic,
  std::string requester_identity = "participant-1")
{
  return IncomingControlPacket{toBytes(command_payload), control_topic, std::move(requester_identity)};
}

struct RoutedSubscriptionHeartbeat final
{
  std::string requester_identity;
  SubscriptionHeartbeat heartbeat;
};

struct RoutedPublishCommand final
{
  std::string requester_identity;
  TopicPublishCommand command;
};

struct RouterProbe final
{
  std::optional<RoutedSubscriptionHeartbeat> heartbeat_call;
  std::optional<RoutedPublishCommand> publish_call;

  ControlPacketRouter makeRouter()
  {
    auto clock = makeTestClock();
    ControlPacketRouter::Callbacks callbacks;
    callbacks.on_subscription_heartbeat = [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
      heartbeat_call = RoutedSubscriptionHeartbeat{std::move(requester_identity), std::move(heartbeat)};
    };
    callbacks.on_topic_publish_command = [this](std::string requester_identity, TopicPublishCommand command) {
      publish_call = RoutedPublishCommand{std::move(requester_identity), std::move(command)};
    };
    return ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"), std::move(clock), std::move(callbacks));
  }
};

RouterProbe routePacket(const IncomingControlPacket & packet)
{
  RouterProbe probe;
  auto router = probe.makeRouter();
  router.route(packet);
  return probe;
}

void expectNoDispatch(const IncomingControlPacket & packet)
{
  const auto probe = routePacket(packet);
  EXPECT_FALSE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
}

}  // namespace

TEST(ControlPacketRouterTest, RoutesHeartbeatPayloadsToTheHeartbeatCallback)
{
  const auto expect_heartbeat_route = [](std::string requester_identity) {
    const auto probe =
      routePacket(makePacket(makeHeartbeatPayload(), protocol::kControlSubscriptionsHeartbeat, requester_identity));

    ASSERT_TRUE(probe.heartbeat_call.has_value());
    EXPECT_FALSE(probe.publish_call.has_value());
    EXPECT_EQ(probe.heartbeat_call->requester_identity, requester_identity);
    ASSERT_TRUE(probe.heartbeat_call->heartbeat.session_id.has_value());
    EXPECT_EQ(*probe.heartbeat_call->heartbeat.session_id, "session-1");
    ASSERT_EQ(probe.heartbeat_call->heartbeat.subscriptions.size(), 1U);
    const auto & subscription = probe.heartbeat_call->heartbeat.subscriptions.front();
    EXPECT_EQ(subscription.target.kind, SubscriptionTargetKind::Topic);
    EXPECT_EQ(subscription.target.name, "/battery");
    ASSERT_TRUE(subscription.preferred_interval_ms.has_value());
    EXPECT_EQ(*subscription.preferred_interval_ms, 125);
  };

  expect_heartbeat_route("participant-1");
  expect_heartbeat_route("");
}

TEST(ControlPacketRouterTest, RoutesPublishPayloads)
{
  const auto probe = routePacket(makePacket(makePublishPayload(), protocol::kControlTopicPublish));

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  ASSERT_TRUE(probe.publish_call.has_value());
  EXPECT_EQ(probe.publish_call->requester_identity, "participant-1");
  EXPECT_EQ(probe.publish_call->command.topic, "/battery/cmd");
  EXPECT_EQ(probe.publish_call->command.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(probe.publish_call->command.cdr_payload, (std::vector<std::uint8_t>{0x01U, 0x02U}));
}

TEST(ControlPacketRouterTest, ContainsHeartbeatCallbackExceptionsAndContinuesRouting)
{
  auto clock = makeTestClock();
  int heartbeat_call_count = 0;
  bool publish_called = false;
  std::optional<SubscriptionHeartbeat> subsequent_heartbeat;

  ControlPacketRouter router(
    rclcpp::get_logger("control_packet_router_test"),
    std::move(clock),
    ControlPacketRouter::Callbacks{
      [&](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
        ++heartbeat_call_count;
        if (heartbeat_call_count == 1) {
          throw std::runtime_error("heartbeat callback failed");
        }
        EXPECT_EQ(requester_identity, "participant-1");
        subsequent_heartbeat = std::move(heartbeat);
      },
      [&](std::string, TopicPublishCommand) { publish_called = true; },
    });

  const auto packet = makePacket(makeHeartbeatPayload(), protocol::kControlSubscriptionsHeartbeat);
  EXPECT_NO_THROW(router.route(packet));
  EXPECT_NO_THROW(router.route(packet));

  EXPECT_EQ(heartbeat_call_count, 2);
  EXPECT_FALSE(publish_called);
  ASSERT_TRUE(subsequent_heartbeat.has_value());
}

TEST(ControlPacketRouterTest, ContainsPublishCallbackExceptionsAndContinuesRouting)
{
  auto clock = makeTestClock();
  int publish_call_count = 0;
  bool heartbeat_called = false;
  std::optional<TopicPublishCommand> subsequent_command;

  ControlPacketRouter router(
    rclcpp::get_logger("control_packet_router_test"),
    std::move(clock),
    ControlPacketRouter::Callbacks{
      [&](std::string, SubscriptionHeartbeat) { heartbeat_called = true; },
      [&](std::string requester_identity, TopicPublishCommand command) {
        ++publish_call_count;
        if (publish_call_count == 1) {
          throw std::runtime_error("publish callback failed");
        }
        EXPECT_EQ(requester_identity, "participant-1");
        subsequent_command = std::move(command);
      },
    });

  const auto packet = makePacket(makePublishPayload(), protocol::kControlTopicPublish);
  EXPECT_NO_THROW(router.route(packet));
  EXPECT_NO_THROW(router.route(packet));

  EXPECT_EQ(publish_call_count, 2);
  EXPECT_FALSE(heartbeat_called);
  ASSERT_TRUE(subsequent_command.has_value());
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
  expectNoDispatch(makePacket(makePublishPayload(), protocol::kControlTopicPublish, ""));
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
      ControlPacketRouter::Callbacks{
        [](std::string, SubscriptionHeartbeat) {},
        [](std::string, TopicPublishCommand) {},
      }),
    std::invalid_argument);
  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      makeTestClock(),
      ControlPacketRouter::Callbacks{
        {},
        [](std::string, TopicPublishCommand) {},
      }),
    std::invalid_argument);

  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      makeTestClock(),
      ControlPacketRouter::Callbacks{
        [](std::string, SubscriptionHeartbeat) {},
        {},
      }),
    std::invalid_argument);
}

}  // namespace livekit_ros2_bridge
