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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "data_stream_registry.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_lease_manager.hpp"
#include "video_stream_registry.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;
constexpr auto kShortHeartbeatLeaseDuration = std::chrono::milliseconds(120);
constexpr auto kLeaseWaitBuffer = std::chrono::milliseconds(40);

sensor_msgs::msg::BatteryState makeBatteryState()
{
  sensor_msgs::msg::BatteryState message;
  message.header.stamp.sec = 3;
  message.header.stamp.nanosec = 4000;
  message.voltage = 48.5F;
  message.temperature = 22.25F;
  message.current = -1.5F;
  message.charge = 18.0F;
  message.capacity = 20.0F;
  message.design_capacity = 21.0F;
  message.percentage = 0.9F;
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  message.cell_voltage = {4.1F, 4.0F};
  return message;
}

sensor_msgs::msg::Image makeRgbImage()
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.header.stamp.nanosec = 2000;
  image.width = 2;
  image.height = 2;
  image.encoding = "rgb8";
  image.step = 6;
  image.data = {
    255,
    0,
    0,
    0,
    255,
    0,
    0,
    0,
    255,
    255,
    255,
    255,
  };
  return image;
}

template <typename MessageT>
MessageT deserializeMessage(const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  serialized.reserve(payload.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::copy(payload.begin(), payload.end(), rcl_msg.buffer);
  rcl_msg.buffer_length = payload.size();

  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

template <typename PublisherT, typename MessageT>
bool publishUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<PublisherT> & publisher,
  const MessageT & message,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    publisher->publish(message);
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

VideoStreamConfig makeConfiguredVideoStreamConfig()
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();
  ConfiguredVideoStreamSource configured_source;
  configured_source.ingress_fragment = "videotestsrc is-live=true pattern=black";
  stream_config.configured_sources.emplace("/sources/front", std::move(configured_source));
  return stream_config;
}

SubscriptionLeaseManager makeLeaseManager(
  rclcpp::Node & node,
  FakeRoomConnection & session,
  DataStreamRegistry & data_stream_registry,
  VideoStreamRegistry * video_stream_registry = nullptr,
  const VideoStreamConfig * video_stream_config = nullptr,
  SubscriptionLeaseManager::Clock::duration heartbeat_lease_duration = std::chrono::seconds(45))
{
  AccessPolicyConfig access_policy_config;
  access_policy_config.subscribe.allow = {"*"};
  return SubscriptionLeaseManager(
    node,
    session,
    AccessPolicy(access_policy_config),
    node.get_clock(),
    data_stream_registry,
    video_stream_registry,
    video_stream_config,
    heartbeat_lease_duration);
}

SubscriptionLeaseManager makeLeaseManager(
  rclcpp::Node & node,
  FakeRoomConnection & session,
  DataStreamRegistry & data_stream_registry,
  AccessPolicy access_policy,
  VideoStreamRegistry * video_stream_registry = nullptr,
  const VideoStreamConfig * video_stream_config = nullptr,
  SubscriptionLeaseManager::Clock::duration heartbeat_lease_duration = std::chrono::seconds(45))
{
  return SubscriptionLeaseManager(
    node,
    session,
    std::move(access_policy),
    node.get_clock(),
    data_stream_registry,
    video_stream_registry,
    video_stream_config,
    heartbeat_lease_duration);
}

SubscriptionDemand makeTopicDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{SubscriptionTargetKind::Topic, name, interval_ms};
}

SubscriptionDemand makeConfiguredSourceDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{SubscriptionTargetKind::ConfiguredSource, name, interval_ms};
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

void sendHeartbeat(
  SubscriptionLeaseManager & manager,
  FakeRoomConnectionState & state,
  const std::string & requester_identity,
  const SubscriptionHeartbeat & heartbeat)
{
  state.published_outgoing_packets.clear();
  manager.handleHeartbeat(requester_identity, heartbeat);
}

nlohmann::json sendHeartbeatAndExtractEnvelope(
  SubscriptionLeaseManager & manager,
  FakeRoomConnectionState & state,
  const std::string & requester_identity,
  const SubscriptionHeartbeat & heartbeat)
{
  sendHeartbeat(manager, state, requester_identity, heartbeat);
  return extractPublishedStatusEnvelope(state, requester_identity);
}

nlohmann::json sendHeartbeatAndExtractStatus(
  SubscriptionLeaseManager & manager,
  FakeRoomConnectionState & state,
  const std::string & requester_identity,
  const SubscriptionHeartbeat & heartbeat)
{
  return extractStatusEntry(sendHeartbeatAndExtractEnvelope(manager, state, requester_identity, heartbeat));
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

class SubscriptionLeaseManagerHeartbeatTest : public ::testing::Test
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

  SubscriptionLeaseManager makeManager(
    AccessPolicy access_policy,
    VideoStreamRegistry * video_stream_registry = nullptr,
    const VideoStreamConfig * video_stream_config = nullptr)
  {
    data_stream_registry_ = std::make_unique<DataStreamRegistry>(*node_, *fake_room_connection_);
    return makeLeaseManager(
      *node_,
      *fake_room_connection_,
      *data_stream_registry_,
      std::move(access_policy),
      video_stream_registry,
      video_stream_config);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakeRoomConnection> fake_room_connection_;
  std::unique_ptr<DataStreamRegistry> data_stream_registry_;
  std::shared_ptr<FakeRoomConnectionState> state_;
  AccessPolicy access_policy_;
};

void expectInvalidArgumentMessage(const std::function<void()> & fn, const char * expected_message)
{
  try {
    fn();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), expected_message);
  } catch (const std::exception & exc) {
    FAIL() << "Expected std::invalid_argument, got: " << exc.what();
  } catch (...) {
    FAIL() << "Expected std::invalid_argument";
  }
}

TEST(SubscriptionLeaseManagerTest, HeartbeatReturnsDeterministicDataTrackForNonVideoTopics)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_data_track_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/state";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);

  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const auto second =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 0)}));

  expectStatusEntry(first, "topic", topic.c_str(), "active");
  EXPECT_EQ(first["interface_type"], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(first["delivery"]["kind"], "data");
  EXPECT_EQ(first["delivery"]["content_type"], wire::protocol::kDataContentTypeCdr);
  EXPECT_EQ(first["delivery"]["interval_ms"], 0);
  EXPECT_EQ(first["delivery"]["track_name"], "ros.data.battery.state");
  EXPECT_EQ(second["delivery"]["track_name"], first["delivery"]["track_name"]);
  EXPECT_EQ(
    session.state->published_data_track_names,
    (std::vector<std::string>{
      first["delivery"]["track_name"].get<std::string>(),
      first["delivery"]["track_name"].get<std::string>(),
    }));
  EXPECT_EQ(
    session.state->unpublished_data_track_names,
    (std::vector<std::string>{first["delivery"]["track_name"].get<std::string>()}));
}

TEST(SubscriptionLeaseManagerTest, PushesRawCdrFramesForDataSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_cdr_delivery_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/send";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  const auto message = makeBatteryState();
  ASSERT_TRUE(
    publishUntil(executor, publisher, message, [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  EXPECT_EQ(session.state->pushed_data_track_frames[0].track_name, track_name);
  EXPECT_EQ(
    deserializeMessage<sensor_msgs::msg::BatteryState>(session.state->pushed_data_track_frames[0].payload), message);
}

TEST(SubscriptionLeaseManagerTest, HeartbeatClampsNegativeRequestedIntervalToZero)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_interval_clamp_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/interval_clamped";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, -25)}));

  EXPECT_EQ(status["delivery"]["interval_ms"], 0);
}

TEST(SubscriptionLeaseManagerTest, HeartbeatStatusOmitsTrackNameUntilFailedPublishRecovers)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_failed_publish_response_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/failed_publish_response";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  int publish_attempt_count = 0;
  session.state->publish_data_track_handler =
    [&publish_attempt_count](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    publish_attempt_count++;
    if (publish_attempt_count == 1) {
      throw std::runtime_error("simulated publish failure");
    }

    auto owner = std::make_shared<int>(publish_attempt_count);
    return std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
  };

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);

  const auto failed =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 500)}));

  expectStatusEntry(failed, "topic", topic.c_str(), "active");
  EXPECT_EQ(failed["interface_type"], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(failed["delivery"]["kind"], "data");
  EXPECT_EQ(failed["delivery"]["interval_ms"], 500);
  EXPECT_TRUE(failed["delivery"]["track_name"].get<std::string>().empty());

  const auto recovered =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 500)}));

  expectStatusEntry(recovered, "topic", topic.c_str(), "active");
  EXPECT_EQ(recovered["interface_type"], "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(recovered["delivery"]["kind"], "data");
  EXPECT_EQ(recovered["delivery"]["interval_ms"], 500);
  EXPECT_EQ(recovered["delivery"]["track_name"], "ros.data.battery.failed_publish_response");
  EXPECT_EQ(publish_attempt_count, 2);
}

TEST(SubscriptionLeaseManagerTest, PruneExpiredLeasesKeepsSharedSubscriptionAndRecomputesInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_prune_shared_interval_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/prune_shared_interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry =
    makeLeaseManager(*node, session, data_stream_registry, nullptr, nullptr, kShortHeartbeatLeaseDuration);

  sendHeartbeat(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 50)}));
  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);

  const auto shared =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 300)}));
  ASSERT_EQ(shared["delivery"]["interval_ms"], 50);
  const auto unpublished_before_prune = session.state->unpublished_data_track_names;

  registry.pruneExpiredLeases();

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, unpublished_before_prune);

  const auto renewed =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 300)}));
  EXPECT_EQ(renewed["delivery"]["interval_ms"], 300);
}

TEST(SubscriptionLeaseManagerTest, OmittedHeartbeatTargetExpiresWhileRenewedSiblingTargetStaysActive)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_omitted_target_expiry_test");
  FakeRoomConnection session;
  const std::string topic_a = "/battery/omitted_stays_alive";
  const std::string topic_b = "/battery/omitted_expires";
  auto publisher_a = node->create_publisher<sensor_msgs::msg::BatteryState>(topic_a, rclcpp::QoS(10));
  auto publisher_b = node->create_publisher<sensor_msgs::msg::BatteryState>(topic_b, rclcpp::QoS(10));
  (void)publisher_a;
  (void)publisher_b;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic_a, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, topic_b, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry =
    makeLeaseManager(*node, session, data_stream_registry, nullptr, nullptr, kShortHeartbeatLeaseDuration);

  sendHeartbeat(
    registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic_a, 0), makeTopicDemand(topic_b, 0)}));

  ASSERT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_a) != nullptr);
  ASSERT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_b) != nullptr);

  const auto renew_delay = kShortHeartbeatLeaseDuration / 2;
  std::this_thread::sleep_for(renew_delay);

  const auto envelope =
    sendHeartbeatAndExtractEnvelope(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic_a, 0)}));
  ASSERT_TRUE(envelope.contains("subscriptions"));
  ASSERT_EQ(envelope["subscriptions"].size(), 1U);

  const auto battery_a_status = findStatusEntry(envelope, "topic", topic_a.c_str());
  ASSERT_TRUE(battery_a_status.has_value());
  expectStatusEntry(*battery_a_status, "topic", topic_a.c_str(), "active");
  EXPECT_FALSE(findStatusEntry(envelope, "topic", topic_b.c_str()).has_value());

  std::this_thread::sleep_for(renew_delay + kLeaseWaitBuffer);
  registry.pruneExpiredLeases();

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_a) != nullptr);
  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_b) != nullptr);
}

TEST(SubscriptionLeaseManagerTest, CreatesVideoSubscriptionsForRosTopicsAndConfiguredSources)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_test");
  FakeRoomConnection session;
  VideoStreamRegistry video_stream_registry(*node, session);
  const VideoStreamConfig video_stream_config = makeConfiguredVideoStreamConfig();
  const std::string video_topic = "/camera/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, video_topic, "sensor_msgs/msg/Image"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry, &video_stream_registry, &video_stream_config);

  const auto topic_status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(video_topic)}));
  const auto source_status = sendHeartbeatAndExtractStatus(
    registry, *session.state, "bob", makeHeartbeat({makeConfiguredSourceDemand("/sources/front")}));

  EXPECT_EQ(topic_status["delivery"]["kind"], "video");
  EXPECT_EQ(topic_status["delivery"]["track_name"], "ros.video.camera.front");
  EXPECT_EQ(source_status["delivery"]["kind"], "video");
  EXPECT_FALSE(source_status["delivery"]["track_name"].get<std::string>().empty());
}

TEST(SubscriptionLeaseManagerTest, ParticipantRefreshRepublishesPublishedDataTrackWithoutDroppingLease)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_republish_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/refresh_replay";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 1000)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();

  registry.onRemoteParticipantDisconnected("alice");
  const auto renewed =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 1000)}));

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(renewed["delivery"]["track_name"], track_name);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
  EXPECT_EQ(session.state->published_data_track_names, (std::vector<std::string>{track_name, track_name}));
}

TEST(SubscriptionLeaseManagerTest, NewRequesterHeartbeatRepublishesAlreadyPublishedDataTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_new_requester_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/new_requester";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 1000)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();
  const auto second =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 250)}));

  EXPECT_EQ(second["delivery"]["track_name"], track_name);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
  EXPECT_EQ(session.state->published_data_track_names, (std::vector<std::string>{track_name, track_name}));
}

TEST(SubscriptionLeaseManagerTest, PruneExpiredLeasesUnpublishesPublishedTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_prune_unpublish_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/prune_expired";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry =
    makeLeaseManager(*node, session, data_stream_registry, nullptr, nullptr, kShortHeartbeatLeaseDuration);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);
  registry.pruneExpiredLeases();

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
}

TEST(SubscriptionLeaseManagerTest, ResetSessionStateClearsDataAndVideoSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_reset_test");
  FakeRoomConnection session;
  VideoStreamRegistry video_stream_registry(*node, session);
  const std::string data_topic = "/battery/reset";
  const std::string video_topic = "/camera/reset";
  auto data_pub = node->create_publisher<sensor_msgs::msg::BatteryState>(data_topic, rclcpp::QoS(10));
  auto video_pub = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)data_pub;
  (void)video_pub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, data_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, video_topic, "sensor_msgs/msg/Image"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry, &video_stream_registry);
  const auto data_status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(data_topic, 0)}));
  const std::string data_track_name = data_status["delivery"]["track_name"].get<std::string>();
  sendHeartbeat(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(video_topic, 0)}));
  ASSERT_TRUE(publishUntil(
    executor, video_pub, makeRgbImage(), [&]() { return session.state->published_video_track_names.size() == 1U; }));

  registry.resetSessionState();

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, data_topic) != nullptr);
  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, video_topic) != nullptr);
  EXPECT_FALSE(data_stream_registry.onTrackPublished(data_track_name, data_stream_registry.generation()));
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{data_track_name});
  EXPECT_EQ(session.state->unpublished_video_track_names, std::vector<std::string>{"ros.video.camera.reset"});
}

TEST(SubscriptionLeaseManagerTest, ShutdownWaitsForActiveSerializedMessageCallback)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_quiesce_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/shutdown_quiesce";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  auto push_entered = std::make_shared<std::promise<void>>();
  auto push_entered_future = push_entered->get_future();
  auto release_push = std::make_shared<std::promise<void>>();
  auto release_push_future = release_push->get_future().share();
  std::atomic<int> push_call_count{0};
  session.state->try_push_data_track_handler = [push_entered, release_push_future, &push_call_count](
                                                 const std::string &, const std::vector<std::uint8_t> &) {
    const int call_number = push_call_count.fetch_add(1) + 1;
    if (call_number == 1) {
      push_entered->set_value();
      release_push_future.wait();
    }
    return DataTrackPushResult::success();
  };

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  std::thread spin_thread([&executor]() { executor.spin(); });

  publisher->publish(makeBatteryState());
  EXPECT_EQ(push_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto shutdown_future = std::async(std::launch::async, [&registry]() { registry.shutdown(); });
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  release_push->set_value();

  EXPECT_EQ(shutdown_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  shutdown_future.get();
  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(push_call_count.load(), 1);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionLeaseManagerTest, QueueFullPushLeavesSubscriptionActive)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_queue_full_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/queue_full";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::atomic<int> push_attempt_count{0};
  session.state->try_push_data_track_handler = [&push_attempt_count](
                                                 const std::string &, const std::vector<std::uint8_t> &) {
    push_attempt_count.fetch_add(1);
    return DataTrackPushResult::failure(DataTrackPushError{DataTrackPushErrorCode::kQueueFull, "queue full"});
  };

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  sendHeartbeat(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return push_attempt_count.load() >= 1; }));
  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
}

TEST(SubscriptionLeaseManagerTest, OnRemoteParticipantDisconnectedRejectsEmptyIdentity)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_requester_test");
  FakeRoomConnection session;
  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);

  expectInvalidArgumentMessage(
    [&registry]() { registry.onRemoteParticipantDisconnected(""); }, "requester_identity is required");
}

TEST(SubscriptionLeaseManagerTest, ShutdownReportsUnavailableSubscriptionsAndFurtherLifecycleCallsAreNoOps)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_rejection_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/shutdown_rejection";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  auto registry = makeLeaseManager(*node, session, data_stream_registry);
  const auto active =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = active["delivery"]["track_name"].get<std::string>();

  registry.shutdown();

  const auto unavailable =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  expectStatusEntry(unavailable, "topic", topic.c_str(), "error");
  EXPECT_EQ(unavailable["error"]["reason"], "unavailable");
  EXPECT_EQ(unavailable["error"]["message"], "Subscription registry is shut down.");

  registry.pruneExpiredLeases();
  registry.resetSessionState();

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, ForbiddenTopicReturnsError)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});

  auto manager = makeManager(deny_all);

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, MissingVideoStreamRegistryReturnsUnavailable)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::Image>(executor, node_, "/camera/front", "sensor_msgs/msg/Image");

  auto manager = makeManager(access_policy_);

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeTopicDemand("/camera/front")}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/camera/front", "unavailable", "Video stream registry is unavailable.");
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, ConfiguredSourceBypassesRosAccessPolicyAndReturnsVideoStatus)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});
  const VideoStreamConfig video_stream_config = makeConfiguredVideoStreamConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_room_connection_);

  auto manager = makeManager(deny_all, &video_stream_registry, &video_stream_config);

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeConfiguredSourceDemand("/sources/front")}));

  const auto status = extractPublishedStatusEntry(*state_, "requester-1");
  expectStatusEntry(status, "configured_source", "/sources/front", "active");
  const auto & delivery = status["delivery"];
  EXPECT_EQ(delivery["kind"], "video");
  EXPECT_FALSE(delivery["track_name"].get<std::string>().empty());
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, MissingConfiguredSourceReturnsErrorOnSourceIdField)
{
  const VideoStreamConfig video_stream_config = makeConfiguredVideoStreamConfig();
  VideoStreamRegistry video_stream_registry(*node_, *fake_room_connection_);

  auto manager = makeManager(access_policy_, &video_stream_registry, &video_stream_config);

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeConfiguredSourceDemand("/sources/missing")}));

  expectPublishedError(
    *state_,
    "requester-1",
    "configured_source",
    "/sources/missing",
    "not_found",
    "Unknown configured video source '/sources/missing'.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, ActiveSubscriptionPublishesSubscriptionStatusEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto manager = makeManager(access_policy_);

  manager.handleHeartbeat(
    "requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

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

TEST_F(SubscriptionLeaseManagerHeartbeatTest, OmittedHeartbeatTargetRemainsActiveUntilItsOriginalLeaseExpires)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto battery_a =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_a", "sensor_msgs/msg/BatteryState");
  auto battery_b =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_b", "sensor_msgs/msg/BatteryState");

  auto manager = makeManager(access_policy_);

  manager.handleHeartbeat(
    "requester-1", makeHeartbeat({makeTopicDemand("/battery_a", 100), makeTopicDemand("/battery_b", 200)}));

  ASSERT_TRUE(manager.findSubscription(SubscriptionTargetKind::Topic, "/battery_a") != nullptr);
  ASSERT_TRUE(manager.findSubscription(SubscriptionTargetKind::Topic, "/battery_b") != nullptr);

  state_->published_outgoing_packets.clear();

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeTopicDemand("/battery_a", 100)}));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  ASSERT_TRUE(envelope.contains("subscriptions"));
  ASSERT_EQ(envelope["subscriptions"].size(), 1U);

  const auto battery_a_status = findStatusEntry(envelope, "topic", "/battery_a");
  ASSERT_TRUE(battery_a_status.has_value());
  expectStatusEntry(*battery_a_status, "topic", "/battery_a", "active");
  EXPECT_FALSE(findStatusEntry(envelope, "topic", "/battery_b").has_value());

  EXPECT_TRUE(manager.findSubscription(SubscriptionTargetKind::Topic, "/battery_a") != nullptr);
  EXPECT_TRUE(manager.findSubscription(SubscriptionTargetKind::Topic, "/battery_b") != nullptr);
  EXPECT_TRUE(state_->unpublished_data_track_names.empty());

  (void)battery_a;
  (void)battery_b;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, AnonymousHeartbeatRenewsKnownClientSession)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto manager = makeManager(access_policy_);

  const auto heartbeat = makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1"));
  manager.handleHeartbeat("requester-1", heartbeat);
  state_->published_outgoing_packets.clear();

  manager.handleHeartbeat("", heartbeat);

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, AnonymousHeartbeatWithoutResolvableClientSessionIsDropped)
{
  auto manager = makeManager(access_policy_);

  manager.handleHeartbeat("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("unknown-session")));
  manager.handleHeartbeat("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  EXPECT_EQ(state_->publish_packet_call_count, 0);
}

TEST_F(
  SubscriptionLeaseManagerHeartbeatTest,
  EmptyHeartbeatBindsClientSessionLeaseThatAnonymousHeartbeatCanStillUseAfterConflict)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto manager = makeManager(access_policy_);

  const auto bind_heartbeat = makeHeartbeat({}, std::string("session-1"));
  manager.handleHeartbeat("requester-1", bind_heartbeat);
  manager.handleHeartbeat("requester-2", bind_heartbeat);

  EXPECT_EQ(state_->publish_packet_call_count, 0);

  manager.handleHeartbeat("", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, CopiesAccessPolicyAtConstruction)
{
  AccessPolicy policy = makeSubscribePolicy({}, {"*"});

  auto manager = makeManager(policy);

  policy = makeSubscribePolicy({"*"});

  manager.handleHeartbeat("requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, PublishControlPacketFailureIsHandledGracefully)
{
  state_->throw_on_publish_packet = true;

  auto manager = makeManager(access_policy_);

  EXPECT_NO_THROW(manager.handleHeartbeat("requester-1", makeHeartbeat({makeTopicDemand("/nonexistent_topic", 100)})));
  EXPECT_EQ(state_->publish_packet_call_count, 1);
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, MixedSubscriptionResultsArePublishedInOneEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher =
    advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state", "sensor_msgs/msg/BatteryState");

  auto manager = makeManager(access_policy_);

  manager.handleHeartbeat(
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
