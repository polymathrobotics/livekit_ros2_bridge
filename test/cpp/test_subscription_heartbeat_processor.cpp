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
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "room_session.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "subscription_registry.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;

VideoConfig makeConfiguredVideoConfig()
{
  VideoConfig config = makeDefaultVideoConfig();

  ConfiguredVideoSourceConfig configured_source_config;
  configured_source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";
  config.configured_sources.emplace("/sources/front", std::move(configured_source_config));
  return config;
}

SubscriptionHeartbeat makeSubscriptionHeartbeat(const nlohmann::json & body)
{
  return parseSubscriptionHeartbeat(body);
}

AccessPolicy makeSubscribePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.subscribe.allow = std::move(allow);
  config.subscribe.deny = std::move(deny);
  return AccessPolicy(config);
}

nlohmann::json extractSinglePublishedStatusEnvelope(
  const FakeRoomSessionState & state, const std::string & requester_identity)
{
  if (state.published_outgoing_control_packets.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_outgoing_control_packets.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_outgoing_control_packets.front();
  EXPECT_EQ(packet.control_topic, protocol::kControlSubscriptionsStatus);

  if (packet.recipient_identities.size() != 1U) {
    ADD_FAILURE() << "Expected one recipient identity, got " << packet.recipient_identities.size();
    return nlohmann::json::object();
  }

  EXPECT_EQ(packet.recipient_identities.front(), requester_identity);
  return nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
}

nlohmann::json extractSinglePublishedStream(const FakeRoomSessionState & state, const std::string & requester_identity)
{
  const auto response = extractSinglePublishedStatusEnvelope(state, requester_identity);
  if (!response.contains("streams") || response["streams"].size() != 1U) {
    ADD_FAILURE() << "Expected one stream entry in status response";
    return nlohmann::json::object();
  }

  return response["streams"].front();
}

class SubscriptionHeartbeatProcessorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_hb_node");
    fake_session_ = std::make_unique<FakeRoomSession>();
    state_ = fake_session_->state;
    access_policy_ = makeSubscribePolicy({"*"});
  }

  SubscriptionRegistry makeRegistry(
    VideoStreamRegistry * video_stream_registry = nullptr, const VideoConfig * video_config = nullptr)
  {
    return SubscriptionRegistry(*node_, *fake_session_, video_stream_registry, video_config);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakeRoomSession> fake_session_;
  std::shared_ptr<FakeRoomSessionState> state_;
  AccessPolicy access_policy_;
};

TEST_F(SubscriptionHeartbeatProcessorTest, EmptyHeartbeatDoesNotBroadcast)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {{"subscriptions", nlohmann::json::array()}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  EXPECT_EQ(state_->publish_control_packet_call_count, 0);
  EXPECT_TRUE(state_->published_outgoing_control_packets.empty());
}

TEST_F(SubscriptionHeartbeatProcessorTest, ForbiddenTopicReturnsError)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, deny_all, node_->get_clock());

  const nlohmann::json body = {
    {"subscriptions", {{{"topic", "/battery_state"}, {"delivery_preferences", {{"interval_ms", 100}}}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "topic");
  EXPECT_EQ(stream["name"], "/battery_state");
  EXPECT_EQ(stream["status"], "error");
  EXPECT_EQ(stream["error"]["reason"], "forbidden");
}

TEST_F(SubscriptionHeartbeatProcessorTest, NotFoundTopicReturnsError)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {
    {"subscriptions", {{{"topic", "/nonexistent_topic"}, {"delivery_preferences", {{"interval_ms", 100}}}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "topic");
  EXPECT_EQ(stream["name"], "/nonexistent_topic");
  EXPECT_EQ(stream["status"], "error");
  EXPECT_EQ(stream["error"]["reason"], "not_found");
}

TEST_F(SubscriptionHeartbeatProcessorTest, MissingVideoStreamRegistryReturnsUnavailable)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = node_->create_publisher<sensor_msgs::msg::Image>("/camera/front", 1);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/camera/front", "sensor_msgs/msg/Image"));

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {{"subscriptions", {{{"topic", "/camera/front"}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "topic");
  EXPECT_EQ(stream["name"], "/camera/front");
  EXPECT_EQ(stream["status"], "error");
  EXPECT_EQ(stream["error"]["reason"], "unavailable");
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, ConfiguredSourceBypassesRosAccessPolicyAndReturnsVideoStatus)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});
  const VideoConfig video_config = makeConfiguredVideoConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_session_);

  auto registry = makeRegistry(&video_stream_registry, &video_config);
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, deny_all, node_->get_clock());

  const nlohmann::json body = {{"subscriptions", {{{"configured_source", "/sources/front"}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "configured_source");
  EXPECT_EQ(stream["name"], "/sources/front");
  EXPECT_EQ(stream["status"], "active");
  EXPECT_EQ(stream["delivery"]["kind"], protocol::kDeliveryKindVideo);
  EXPECT_EQ(stream["delivery"]["track_name"], "ros.video.configured_source.%2Fsources%2Ffront");
  EXPECT_FALSE(stream.contains("error"));
}

TEST_F(SubscriptionHeartbeatProcessorTest, MissingConfiguredSourceReturnsErrorOnSourceIdField)
{
  const VideoConfig video_config = makeConfiguredVideoConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_session_);

  auto registry = makeRegistry(&video_stream_registry, &video_config);
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {{"subscriptions", {{{"configured_source", "/sources/missing"}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "configured_source");
  EXPECT_EQ(stream["name"], "/sources/missing");
  EXPECT_EQ(stream["status"], "error");
  EXPECT_EQ(stream["error"]["reason"], "not_found");
}

TEST_F(SubscriptionHeartbeatProcessorTest, ActiveSubscriptionPublishesStreamStatusEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 1);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState"));

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {
    {"session_id", "session-1"},
    {"subscriptions", {{{"topic", "/battery_state"}, {"delivery_preferences", {{"interval_ms", 100}}}}}},
  };
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto response = extractSinglePublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(response["v"], protocol::kProtocolVersion);
  EXPECT_EQ(response["type"], protocol::kControlSubscriptionsStatus);
  EXPECT_EQ(response["session_id"], "session-1");
  ASSERT_TRUE(response["lease_expires_in_ms"].is_number_integer());
  EXPECT_GT(response["lease_expires_in_ms"].get<std::int64_t>(), 0);
  ASSERT_EQ(response["streams"].size(), 1U);
  EXPECT_EQ(response["streams"][0]["kind"], "topic");
  EXPECT_EQ(response["streams"][0]["name"], "/battery_state");
  EXPECT_EQ(response["streams"][0]["status"], "active");
  EXPECT_EQ(response["streams"][0]["interface_type"], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(response["streams"][0]["delivery"]["kind"], protocol::kDeliveryKindData);
  EXPECT_EQ(response["streams"][0]["delivery"]["track_name"], "ros.data.battery_state");
  EXPECT_EQ(response["streams"][0]["delivery"]["content_type"], "application/x-ros-cdr");
  EXPECT_EQ(response["streams"][0]["delivery"]["interval_ms"], 100);
  EXPECT_FALSE(response["streams"][0]["delivery"].contains("publisher_identity"));
  EXPECT_FALSE(response["streams"][0].contains("error"));
  EXPECT_FALSE(response["streams"][0].contains("target"));
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, AnonymousHeartbeatRenewsKnownSession)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 1);
  ASSERT_TRUE(waitForTopicType(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState"));

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {
    {"session_id", "session-1"},
    {"subscriptions", {{{"topic", "/battery_state"}, {"delivery_preferences", {{"interval_ms", 100}}}}}},
  };
  processor.process("requester-1", makeSubscriptionHeartbeat(body));
  state_->published_outgoing_control_packets.clear();

  processor.process("", makeSubscriptionHeartbeat(body));

  const auto response = extractSinglePublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(response["session_id"], "session-1");
  ASSERT_EQ(response["streams"].size(), 1U);
  EXPECT_EQ(response["streams"][0]["name"], "/battery_state");
  EXPECT_EQ(response["streams"][0]["status"], "active");
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, AnonymousHeartbeatWithoutKnownSessionIsDropped)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {
    {"session_id", "unknown-session"},
    {"subscriptions", {{{"topic", "/battery_state"}, {"delivery_preferences", {{"interval_ms", 100}}}}}},
  };

  processor.process("", makeSubscriptionHeartbeat(body));

  EXPECT_EQ(state_->publish_control_packet_call_count, 0);
  EXPECT_TRUE(state_->published_outgoing_control_packets.empty());
}

TEST_F(SubscriptionHeartbeatProcessorTest, SessionConflictDoesNotRebindRequesterIdentity)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const auto body = makeSubscriptionHeartbeat(
    nlohmann::json{
      {"session_id", "session-1"},
      {"subscriptions", nlohmann::json::array()},
    });

  processor.process("requester-1", body);
  state_->published_outgoing_control_packets.clear();
  const int publish_count_after_bind = state_->publish_control_packet_call_count;
  processor.process("requester-2", body);

  EXPECT_EQ(state_->publish_control_packet_call_count, publish_count_after_bind);
  EXPECT_TRUE(state_->published_outgoing_control_packets.empty());
}

TEST_F(SubscriptionHeartbeatProcessorTest, CopiesAccessPolicyAtConstruction)
{
  AccessPolicy policy = makeSubscribePolicy({}, {"*"});

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, policy, node_->get_clock());

  policy = makeSubscribePolicy({"*"});

  const nlohmann::json body = {
    {"subscriptions", {{{"topic", "/battery_state"}, {"delivery_preferences", {{"interval_ms", 100}}}}}}};
  processor.process("requester-1", makeSubscriptionHeartbeat(body));

  const auto stream = extractSinglePublishedStream(*state_, "requester-1");
  EXPECT_EQ(stream["kind"], "topic");
  EXPECT_EQ(stream["name"], "/battery_state");
  EXPECT_EQ(stream["status"], "error");
  EXPECT_EQ(stream["error"]["reason"], "forbidden");
}

TEST_F(SubscriptionHeartbeatProcessorTest, PublishControlPacketFailureIsHandledGracefully)
{
  state_->throw_on_publish_control_packet = true;

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_session_, access_policy_, node_->get_clock());

  const nlohmann::json body = {
    {"subscriptions", {{{"topic", "/nonexistent_topic"}, {"delivery_preferences", {{"interval_ms", 100}}}}}}};

  EXPECT_NO_THROW(processor.process("requester-1", makeSubscriptionHeartbeat(body)));
  EXPECT_EQ(state_->publish_control_packet_call_count, 1);
  EXPECT_TRUE(state_->published_outgoing_control_packets.empty());
}

}  // namespace
}  // namespace livekit_ros2_bridge
