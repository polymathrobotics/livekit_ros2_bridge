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
#include <string>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "subscription_registry.hpp"
#include "video_stream_registry.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;

VideoStreamConfig makeConfiguredVideoStreamConfig()
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  ConfiguredVideoStreamSource configured_source;
  configured_source.ingress_fragment = "videotestsrc is-live=true pattern=black";
  stream_config.configured_sources.emplace("/sources/front", std::move(configured_source));
  return stream_config;
}

SubscriptionDemand makeTopicDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{{SubscriptionTargetKind::Topic, name}, interval_ms};
}

SubscriptionDemand makeConfiguredSourceDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{{SubscriptionTargetKind::ConfiguredSource, name}, interval_ms};
}

SubscriptionHeartbeat makeHeartbeat(
  std::vector<SubscriptionDemand> demands, std::optional<std::string> session_id = std::nullopt)
{
  SubscriptionHeartbeat heartbeat;
  heartbeat.session_id = std::move(session_id);
  heartbeat.subscriptions = std::move(demands);
  return heartbeat;
}

AccessPolicy makeSubscribePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.subscribe.allow = std::move(allow);
  config.subscribe.deny = std::move(deny);
  return AccessPolicy(config);
}

template <typename MessageT>
std::shared_ptr<rclcpp::Publisher<MessageT>> advertiseTopic(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic,
  const std::string & interface_type)
{
  auto publisher = node->create_publisher<MessageT>(topic, 1);
  EXPECT_TRUE(waitForTopicType(executor, node, topic, interface_type));
  return publisher;
}

nlohmann::json extractPublishedStatusEnvelope(
  const FakeRoomConnectionState & state, const std::string & requester_identity)
{
  if (state.published_outgoing_packets.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_outgoing_packets.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_outgoing_packets.front();
  EXPECT_EQ(packet.topic, wire::protocol::kControlSubscriptionsStatus);

  if (packet.recipient_identities.size() != 1U) {
    ADD_FAILURE() << "Expected one recipient identity, got " << packet.recipient_identities.size();
    return nlohmann::json::object();
  }

  EXPECT_EQ(packet.recipient_identities.front(), requester_identity);
  return nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
}

nlohmann::json extractStatusEntry(const nlohmann::json & envelope)
{
  if (!envelope.contains("subscriptions") || envelope["subscriptions"].size() != 1U) {
    ADD_FAILURE() << "Expected one subscription status object in status response";
    return nlohmann::json::object();
  }

  return envelope["subscriptions"].front();
}

nlohmann::json extractPublishedStatusEntry(
  const FakeRoomConnectionState & state, const std::string & requester_identity)
{
  return extractStatusEntry(extractPublishedStatusEnvelope(state, requester_identity));
}

std::optional<nlohmann::json> findStatusEntry(const nlohmann::json & envelope, const char * kind, const char * name)
{
  if (!envelope.contains("subscriptions") || !envelope["subscriptions"].is_array()) {
    return std::nullopt;
  }

  for (const auto & status : envelope["subscriptions"]) {
    if (status.value("kind", std::string()) == kind && status.value("name", std::string()) == name) {
      return std::optional<nlohmann::json>{status};
    }
  }

  return std::nullopt;
}

void expectStatusEntry(const nlohmann::json & status, const char * kind, const char * name, const char * value)
{
  EXPECT_EQ(status["kind"], kind);
  EXPECT_EQ(status["name"], name);
  EXPECT_EQ(status["status"], value);
}

void expectPublishedError(
  const FakeRoomConnectionState & state,
  const std::string & requester_identity,
  const char * kind,
  const char * name,
  const char * reason,
  const char * message)
{
  const auto status = extractPublishedStatusEntry(state, requester_identity);
  expectStatusEntry(status, kind, name, "error");
  EXPECT_EQ(status["error"]["reason"], reason);
  EXPECT_EQ(status["error"]["message"], message);
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
    fake_room_connection_ = std::make_unique<FakeRoomConnection>();
    state_ = fake_room_connection_->state;
    access_policy_ = makeSubscribePolicy({"*"});
  }

  SubscriptionRegistry makeRegistry(
    VideoStreamRegistry * video_stream_registry = nullptr, const VideoStreamConfig * video_stream_config = nullptr)
  {
    return SubscriptionRegistry(*node_, *fake_room_connection_, video_stream_registry, video_stream_config);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakeRoomConnection> fake_room_connection_;
  std::shared_ptr<FakeRoomConnectionState> state_;
  AccessPolicy access_policy_;
};

TEST_F(SubscriptionHeartbeatProcessorTest, ForbiddenTopicReturnsError)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, deny_all, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionHeartbeatProcessorTest, NotFoundTopicReturnsError)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeTopicDemand("/nonexistent_topic", 100)}));

  expectPublishedError(
    *state_,
    "requester-1",
    "topic",
    "/nonexistent_topic",
    "not_found",
    "No ROS types found for topic '/nonexistent_topic'.");
}

TEST_F(SubscriptionHeartbeatProcessorTest, MissingVideoStreamRegistryReturnsUnavailable)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::Image>(executor, node_, "/camera/front", "sensor_msgs/msg/Image");

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeTopicDemand("/camera/front")}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/camera/front", "unavailable", "Video stream registry is unavailable.");
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, ConfiguredSourceBypassesRosAccessPolicyAndReturnsVideoStatus)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});
  const VideoStreamConfig video_stream_config = makeConfiguredVideoStreamConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_room_connection_);

  auto registry = makeRegistry(&video_stream_registry, &video_stream_config);
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, deny_all, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeConfiguredSourceDemand("/sources/front")}));

  const auto status = extractPublishedStatusEntry(*state_, "requester-1");
  expectStatusEntry(status, "configured_source", "/sources/front", "active");
  const auto & delivery = status["delivery"];
  EXPECT_EQ(delivery["kind"], "video");
  EXPECT_FALSE(delivery["track_name"].get<std::string>().empty());
}

TEST_F(SubscriptionHeartbeatProcessorTest, MissingConfiguredSourceReturnsErrorOnSourceIdField)
{
  const VideoStreamConfig video_stream_config = makeConfiguredVideoStreamConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_room_connection_);

  auto registry = makeRegistry(&video_stream_registry, &video_stream_config);
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeConfiguredSourceDemand("/sources/missing")}));

  expectPublishedError(
    *state_,
    "requester-1",
    "configured_source",
    "/sources/missing",
    "not_found",
    "Unknown configured video source '/sources/missing'.");
}

TEST_F(SubscriptionHeartbeatProcessorTest, ActiveSubscriptionPublishesSubscriptionStatusEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process("requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["v"], wire::protocol::kProtocolVersion);
  EXPECT_EQ(envelope["type"], wire::protocol::kControlSubscriptionsStatus);
  EXPECT_EQ(envelope["session_id"], "session-1");
  ASSERT_TRUE(envelope["lease_expires_in_ms"].is_number_integer());
  EXPECT_GT(envelope["lease_expires_in_ms"].get<std::int64_t>(), 0);

  const auto status = extractStatusEntry(envelope);
  const auto & delivery = status["delivery"];
  expectStatusEntry(status, "topic", "/battery_state", "active");
  EXPECT_EQ(status["interface_type"], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(delivery["track_name"], "ros.data.battery_state");
  EXPECT_EQ(delivery["interval_ms"], 100);
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, AnonymousHeartbeatRenewsKnownClientSession)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  const auto heartbeat = makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1"));
  processor.process("requester-1", heartbeat);
  state_->published_outgoing_packets.clear();

  processor.process("", heartbeat);

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, AnonymousHeartbeatWithoutResolvableClientSessionIsDropped)
{
  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("unknown-session")));

  EXPECT_EQ(state_->publish_packet_call_count, 0);

  processor.process("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  EXPECT_EQ(state_->publish_packet_call_count, 0);
}

TEST_F(
  SubscriptionHeartbeatProcessorTest,
  EmptyHeartbeatBindsClientSessionLeaseThatAnonymousHeartbeatCanStillUseAfterConflict)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  const auto bind_heartbeat = makeHeartbeat({}, std::string("session-1"));
  processor.process("requester-1", bind_heartbeat);

  EXPECT_EQ(state_->publish_packet_call_count, 0);

  processor.process("requester-2", bind_heartbeat);

  EXPECT_EQ(state_->publish_packet_call_count, 0);

  processor.process("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  (void)publisher;
}

TEST_F(SubscriptionHeartbeatProcessorTest, CopiesAccessPolicyAtConstruction)
{
  AccessPolicy policy = makeSubscribePolicy({}, {"*"});

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, policy, node_->get_clock());

  policy = makeSubscribePolicy({"*"});

  processor.process("requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionHeartbeatProcessorTest, PublishControlPacketFailureIsHandledGracefully)
{
  state_->throw_on_publish_packet = true;

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  EXPECT_NO_THROW(processor.process("requester-1", makeHeartbeat({makeTopicDemand("/nonexistent_topic", 100)})));
  EXPECT_EQ(state_->publish_packet_call_count, 1);
}

TEST_F(SubscriptionHeartbeatProcessorTest, MixedSubscriptionResultsArePublishedInOneEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto registry = makeRegistry();
  SubscriptionHeartbeatProcessor processor(registry, *fake_room_connection_, access_policy_, node_->get_clock());

  processor.process(
    "requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100), makeTopicDemand("/nonexistent_topic", 100)}));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  ASSERT_TRUE(envelope.contains("subscriptions"));
  ASSERT_EQ(envelope["subscriptions"].size(), 2U);

  const auto active_status = findStatusEntry(envelope, "topic", "/battery_state");
  ASSERT_TRUE(active_status.has_value());
  expectStatusEntry(*active_status, "topic", "/battery_state", "active");

  const auto missing_status = findStatusEntry(envelope, "topic", "/nonexistent_topic");
  ASSERT_TRUE(missing_status.has_value());
  EXPECT_EQ(
    *missing_status,
    nlohmann::json(
      {{"kind", "topic"},
       {"name", "/nonexistent_topic"},
       {"status", "error"},
       {"error", {{"reason", "not_found"}, {"message", "No ROS types found for topic '/nonexistent_topic'."}}}}));
  (void)publisher;
}

}  // namespace
}  // namespace livekit_ros2_bridge
