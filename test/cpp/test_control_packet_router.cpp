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
  return R"({"session_id":"session-1","subscriptions":[{"topic":"/battery","delivery_preferences":{"interval_ms":125}}]})";
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

}  // namespace

TEST(ControlPacketRouterTest, RoutesHeartbeatPayloads)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  const auto heartbeat_packet = makePacket(makeHeartbeatPayload(), protocol::kControlSubscriptionsHeartbeat);
  router.route(heartbeat_packet);

  ASSERT_TRUE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
  EXPECT_EQ(probe.heartbeat_call->requester_identity, "participant-1");
  ASSERT_EQ(probe.heartbeat_call->heartbeat.subscriptions.size(), 1U);
  EXPECT_EQ(probe.heartbeat_call->heartbeat.subscriptions[0].target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(probe.heartbeat_call->heartbeat.subscriptions[0].target.name, "/battery");
  ASSERT_TRUE(probe.heartbeat_call->heartbeat.subscriptions[0].preferred_interval_ms.has_value());
  EXPECT_EQ(*probe.heartbeat_call->heartbeat.subscriptions[0].preferred_interval_ms, 125);
  ASSERT_TRUE(probe.heartbeat_call->heartbeat.session_id.has_value());
  EXPECT_EQ(*probe.heartbeat_call->heartbeat.session_id, "session-1");
}

TEST(ControlPacketRouterTest, RoutesPublishPayloads)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  router.route(makePacket(makePublishPayload(), protocol::kControlTopicPublish));

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  ASSERT_TRUE(probe.publish_call.has_value());
  EXPECT_EQ(probe.publish_call->requester_identity, "participant-1");
  EXPECT_EQ(probe.publish_call->command.topic, "/battery/cmd");
  EXPECT_EQ(probe.publish_call->command.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(probe.publish_call->command.cdr_payload, (std::vector<std::uint8_t>{0x01U, 0x02U}));
}

TEST(ControlPacketRouterTest, RoutesHeartbeatWithoutRequesterIdentity)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  const auto anonymous_heartbeat_packet =
    makePacket(makeHeartbeatPayload(), protocol::kControlSubscriptionsHeartbeat, "");
  router.route(anonymous_heartbeat_packet);

  ASSERT_TRUE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
  EXPECT_EQ(probe.heartbeat_call->requester_identity, "");
  ASSERT_TRUE(probe.heartbeat_call->heartbeat.session_id.has_value());
  EXPECT_EQ(*probe.heartbeat_call->heartbeat.session_id, "session-1");
}

TEST(ControlPacketRouterTest, DoesNotRouteHeartbeatParseFailures)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  router.route(makePacket("{", protocol::kControlSubscriptionsHeartbeat));

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
}

TEST(ControlPacketRouterTest, DropsPublishWithoutRequesterIdentity)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  router.route(makePacket(makePublishPayload(), protocol::kControlTopicPublish, ""));

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
}

TEST(ControlPacketRouterTest, DoesNotRoutePublishParseFailures)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  router.route(makePacket("{", protocol::kControlTopicPublish));

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
}

TEST(ControlPacketRouterTest, IgnoresUnknownTopics)
{
  RouterProbe probe;
  auto router = probe.makeRouter();

  router.route(IncomingControlPacket{{'x'}, "custom.topic", "participant-1"});

  EXPECT_FALSE(probe.heartbeat_call.has_value());
  EXPECT_FALSE(probe.publish_call.has_value());
}

TEST(ControlPacketRouterTest, RequiresHeartbeatCallback)
{
  EXPECT_THROW(
    ControlPacketRouter(
      rclcpp::get_logger("control_packet_router_test"),
      makeTestClock(),
      ControlPacketRouter::Callbacks{
        {},
        [](std::string, TopicPublishCommand) {},
      }),
    std::invalid_argument);
}

TEST(ControlPacketRouterTest, RequiresPublishCallback)
{
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
