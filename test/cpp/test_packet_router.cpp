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

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "data_stream_registry.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "packet_router.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "ros_topic_publisher.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "subscription_lease_manager.hpp"
#include "video_stream_registry.hpp"
#include "wire/cdr.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitForTopicType;

std::string nextNodeName(const char * prefix)
{
  static std::atomic<int> next_id{1};
  return std::string(prefix) + "_" + std::to_string(next_id.fetch_add(1));
}

rclcpp::Clock::SharedPtr makeTestClock()
{
  return std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}

AccessPolicy makeAccessPolicy(
  std::vector<std::string> publish_allow = {}, std::vector<std::string> subscribe_allow = {})
{
  AccessPolicyConfig config;
  config.publish.allow = std::move(publish_allow);
  config.subscribe.allow = std::move(subscribe_allow);
  return AccessPolicy(config);
}

std::vector<std::uint8_t> toBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

template <typename MessageT>
std::vector<std::uint8_t> serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_message = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_message.buffer, rcl_message.buffer + rcl_message.buffer_length);
}

IncomingPacket makePacket(
  const std::string & payload, const std::string & topic, std::string requester_identity = "participant-1")
{
  return IncomingPacket{toBytes(payload), topic, std::move(requester_identity)};
}

std::string heartbeatPayload()
{
  return R"({"session_id":"session-1","subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}}]})";
}

std::string publishPayload()
{
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  return nlohmann::json{
    {"topic", "/battery/cmd"},
    {"interface_type", "sensor_msgs/msg/BatteryState"},
    {"message", wire::cdr::serialize(serializeMessage(message))},
  }
    .dump();
}

nlohmann::json extractSinglePublishedStatusEnvelope(
  const FakeRoomConnectionState & state, const std::string & requester_identity)
{
  if (state.published_outgoing_packets.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_outgoing_packets.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_outgoing_packets.front();
  EXPECT_EQ(packet.topic, wire::protocol::kSubscriptionsStatusTopic);
  EXPECT_EQ(packet.recipient_identities, (std::vector<std::string>{requester_identity}));
  return nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
}

class PacketRouterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>(nextNodeName("packet_router_test"));
    room_connection_ = std::make_unique<FakeRoomConnection>();
  }

  void initRouter(const AccessPolicy & access_policy)
  {
    data_stream_registry_ = std::make_unique<DataStreamRegistry>(*node_, *room_connection_);
    video_stream_registry_ = std::make_unique<VideoStreamRegistry>(*node_, *room_connection_);
    subscription_lease_manager_ = std::make_unique<SubscriptionLeaseManager>(
      *node_, *room_connection_, access_policy, *data_stream_registry_, *video_stream_registry_);
    ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(*node_, access_policy);
    packet_router_ = std::make_unique<PacketRouter>(
      node_->get_clock(),
      [](std::function<void()> work) { work(); },
      *subscription_lease_manager_,
      *ros_topic_publisher_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakeRoomConnection> room_connection_;
  std::unique_ptr<DataStreamRegistry> data_stream_registry_;
  std::unique_ptr<VideoStreamRegistry> video_stream_registry_;
  std::unique_ptr<SubscriptionLeaseManager> subscription_lease_manager_;
  std::unique_ptr<RosTopicPublisher> ros_topic_publisher_;
  std::unique_ptr<PacketRouter> packet_router_;
};

TEST_F(PacketRouterTest, RoutesHeartbeatPacketsViaSubscriptionLeaseManager)
{
  initRouter(makeAccessPolicy({}, {"/battery"}));

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("packet_router_observer"));
  [[maybe_unused]] auto publisher =
    observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(1));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery", "sensor_msgs/msg/BatteryState"));

  EXPECT_NO_THROW(packet_router_->handle(makePacket(heartbeatPayload(), wire::protocol::kSubscriptionsHeartbeatTopic)));

  ASSERT_EQ(room_connection_->state->published_data_track_names.size(), 1U);
  const auto envelope = extractSinglePublishedStatusEnvelope(*room_connection_->state, "participant-1");
  ASSERT_TRUE(envelope.contains("subscriptions"));
  ASSERT_EQ(envelope["subscriptions"].size(), 1U);
  EXPECT_EQ(envelope["session_id"], "session-1");
}

TEST_F(PacketRouterTest, RoutesPublishPacketsViaRosTopicPublisher)
{
  initRouter(makeAccessPolicy({"/battery/cmd"}));

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("packet_router_publish_observer"));
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/cmd", rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery/cmd", "sensor_msgs/msg/BatteryState"));

  EXPECT_NO_THROW(packet_router_->handle(makePacket(publishPayload(), wire::protocol::kTopicPubTopic)));

  ASSERT_TRUE(spinUntil(executor, [&received_message]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
}

TEST_F(PacketRouterTest, RejectsInvalidHeartbeatPayloadsWithoutDispatch)
{
  initRouter(makeAccessPolicy({}, {"/battery"}));

  const auto expect_rejected_heartbeat = [this](const std::string & payload) {
    EXPECT_NO_THROW(packet_router_->handle(makePacket(payload, wire::protocol::kSubscriptionsHeartbeatTopic)));
    EXPECT_TRUE(room_connection_->state->published_outgoing_packets.empty());
    EXPECT_TRUE(room_connection_->state->published_data_track_names.empty());
  };

  expect_rejected_heartbeat("{");
  expect_rejected_heartbeat(R"({})");
}

TEST_F(PacketRouterTest, RejectsInvalidPublishPacketsWithoutDispatch)
{
  initRouter(makeAccessPolicy({"/battery/cmd"}));

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("packet_router_invalid_publish_observer"));
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/cmd", rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery/cmd", "sensor_msgs/msg/BatteryState"));

  const auto expect_rejected_publish = [this, &executor, &received_message](
                                         const std::string & payload,
                                         const std::string & requester_identity = "participant-1") {
    EXPECT_NO_THROW(packet_router_->handle(makePacket(payload, wire::protocol::kTopicPubTopic, requester_identity)));

    executor.spin_some();
    EXPECT_FALSE(received_message.has_value());
  };

  expect_rejected_publish("{");
  expect_rejected_publish(publishPayload(), "");
}

TEST_F(PacketRouterTest, DropsLegacyPublishTopicWithoutDispatch)
{
  initRouter(makeAccessPolicy({"/battery/cmd"}));

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("packet_router_legacy_publish_observer"));
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/cmd", rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery/cmd", "sensor_msgs/msg/BatteryState"));

  EXPECT_NO_THROW(packet_router_->handle(makePacket(publishPayload(), "ros.topics.publish")));

  executor.spin_some();
  EXPECT_FALSE(received_message.has_value());
}

TEST_F(PacketRouterTest, DropsUnsupportedTopicsWithoutDispatch)
{
  initRouter(makeAccessPolicy({"/battery/cmd"}, {"/battery"}));

  EXPECT_NO_THROW(packet_router_->handle(IncomingPacket{{'x'}, "custom.topic", "participant-1"}));
  EXPECT_TRUE(room_connection_->state->published_outgoing_packets.empty());
  EXPECT_TRUE(room_connection_->state->published_data_track_names.empty());
}

TEST_F(PacketRouterTest, ValidatesConstructorDependencies)
{
  const AccessPolicy access_policy = makeAccessPolicy();
  data_stream_registry_ = std::make_unique<DataStreamRegistry>(*node_, *room_connection_);
  video_stream_registry_ = std::make_unique<VideoStreamRegistry>(*node_, *room_connection_);
  subscription_lease_manager_ = std::make_unique<SubscriptionLeaseManager>(
    *node_, *room_connection_, access_policy, *data_stream_registry_, *video_stream_registry_);
  ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(*node_, access_policy);

  EXPECT_THROW(
    PacketRouter(
      {}, [](std::function<void()> work) { work(); }, *subscription_lease_manager_, *ros_topic_publisher_),
    std::invalid_argument);
  EXPECT_THROW(
    PacketRouter(makeTestClock(), {}, *subscription_lease_manager_, *ros_topic_publisher_), std::invalid_argument);
}

}  // namespace
}  // namespace livekit_ros2_bridge
