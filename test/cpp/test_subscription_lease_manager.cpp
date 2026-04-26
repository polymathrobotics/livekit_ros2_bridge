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
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/constants.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_lease_manager.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitForTopicType;
using test_support::waitUntil;
constexpr auto kShortHeartbeatLeaseDuration = std::chrono::milliseconds(120);
constexpr auto kLeaseWaitBuffer = std::chrono::milliseconds(40);

template <typename MessageT>
bool waitForTopic(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic,
  std::chrono::milliseconds timeout = test_support::kDefaultWaitTimeout)
{
  return waitForTopicType(executor, node, topic, rosidl_generator_traits::name<MessageT>(), timeout);
}

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

sensor_msgs::msg::Image makeRgbImage(std::uint32_t width = 2, std::uint32_t height = 2)
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.header.stamp.nanosec = 2000;
  image.width = width;
  image.height = height;
  image.encoding = "rgb8";
  image.step = width * 3U;
  image.data.resize(static_cast<std::size_t>(image.step) * height);
  for (std::size_t idx = 0; idx < image.data.size(); idx += 3U) {
    image.data[idx] = 255;
    if (idx + 1U < image.data.size()) {
      image.data[idx + 1U] = 255;
    }
    if (idx + 2U < image.data.size()) {
      image.data[idx + 2U] = 255;
    }
  }
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

video::StreamConfig makeOtherSourceConfig()
{
  video::StreamConfig config = video::makeDefaultConfig();
  video::OtherSource source;
  source.source_fragment = "videotestsrc is-live=true pattern=black";
  config.other_sources.emplace("/sources/front", std::move(source));
  return config;
}

SubscriptionLeaseManager makeLeaseManager(
  rclcpp::Node & node,
  FakeRoomConnection & session,
  AccessPolicy access_policy,
  const video::StreamConfig * video_config = nullptr,
  SubscriptionLeaseManager::Clock::duration heartbeat_lease_duration = std::chrono::seconds(45))
{
  return SubscriptionLeaseManager(
    node.get_node_parameters_interface(),
    node.get_node_topics_interface(),
    node.get_node_graph_interface(),
    node.get_clock(),
    session,
    std::move(access_policy),
    nullptr,
    video_config,
    heartbeat_lease_duration);
}

SubscriptionLeaseManager makeLeaseManager(
  rclcpp::Node & node,
  FakeRoomConnection & session,
  const video::StreamConfig * video_config = nullptr,
  SubscriptionLeaseManager::Clock::duration heartbeat_lease_duration = std::chrono::seconds(45))
{
  AccessPolicyConfig access_policy_config;
  access_policy_config.subscribe.allow = {"*"};
  return makeLeaseManager(node, session, AccessPolicy(access_policy_config), video_config, heartbeat_lease_duration);
}

SubscriptionDemand makeTopicDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{SubscriptionTargetKind::Topic, name, interval_ms};
}

SubscriptionDemand makeOtherVideoDemand(const std::string & name, std::optional<int> interval_ms = std::nullopt)
{
  return SubscriptionDemand{SubscriptionTargetKind::OtherVideo, name, interval_ms};
}

SubscriptionHeartbeat makeHeartbeat(
  std::vector<SubscriptionDemand> demands, std::optional<std::string> session_id = std::nullopt)
{
  SubscriptionHeartbeat heartbeat;
  heartbeat.session_id = std::move(session_id);
  heartbeat.demands = std::move(demands);
  return heartbeat;
}

std::vector<std::uint8_t> payloadBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

std::vector<std::uint8_t> heartbeatPayloadBytes(const SubscriptionHeartbeat & heartbeat)
{
  nlohmann::json body = {
    {"subscriptions", nlohmann::json::array()},
  };
  if (heartbeat.session_id.has_value()) {
    body["session_id"] = *heartbeat.session_id;
  }

  for (const auto & demand : heartbeat.demands) {
    nlohmann::json entry = {
      {"kind", demand.kind == SubscriptionTargetKind::OtherVideo ? "other_video" : "topic"},
      {"name", demand.name},
    };
    if (demand.preferred_interval_ms.has_value()) {
      entry["delivery_preferences"] = {{"interval_ms", *demand.preferred_interval_ms}};
    }
    body["subscriptions"].push_back(std::move(entry));
  }

  return payloadBytes(body.dump());
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
  const std::string & topic)
{
  auto publisher = node->create_publisher<MessageT>(topic, 1);
  EXPECT_TRUE(waitForTopic<MessageT>(executor, node, topic));
  return publisher;
}

nlohmann::json extractPublishedStatusEnvelope(
  const FakeRoomConnectionState & state, const std::string & requester_identity)
{
  if (state.published_data_calls.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_data_calls.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_data_calls.front();
  EXPECT_EQ(packet.topic, protocol::kStatusTopic);

  if (packet.destination_identities.size() != 1U) {
    ADD_FAILURE() << "Expected one recipient identity, got " << packet.destination_identities.size();
    return nlohmann::json::object();
  }

  EXPECT_EQ(packet.destination_identities.front(), requester_identity);
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
  state.published_data_calls.clear();
  manager.handleHeartbeatPayload(requester_identity, heartbeatPayloadBytes(heartbeat));
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

  SubscriptionLeaseManager makeManager(AccessPolicy access_policy, const video::StreamConfig * video_config = nullptr)
  {
    return makeLeaseManager(*node_, *fake_room_connection_, std::move(access_policy), video_config);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FakeRoomConnection> fake_room_connection_;
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

TEST_F(SubscriptionLeaseManagerHeartbeatTest, HeartbeatPayloadParsesAndHandlesSubscriptions)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state");

  auto manager = makeManager(access_policy_);
  const auto payload = payloadBytes(
    R"({"session_id":"session-1","subscriptions":[{"kind":"topic","name":"/battery_state","delivery_preferences":{"interval_ms":125}}]})");

  EXPECT_NO_THROW(manager.handleHeartbeatPayload("requester-1", payload));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  EXPECT_EQ(status["delivery"]["interval_ms"], 125);
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, InvalidHeartbeatPayloadIsDroppedWithoutDispatch)
{
  auto manager = makeManager(access_policy_);

  EXPECT_NO_THROW(manager.handleHeartbeatPayload("requester-1", payloadBytes("{")));

  EXPECT_EQ(state_->publish_data_call_count, 0);
  EXPECT_TRUE(state_->published_data_track_names.empty());
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);

  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const auto second =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 0)}));

  expectStatusEntry(first, "topic", topic.c_str(), "active");
  EXPECT_EQ(first["interface_type"], rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>());
  EXPECT_EQ(first["delivery"]["kind"], "data");
  EXPECT_EQ(first["delivery"]["content_type"], protocol::kCdrContentType);
  EXPECT_EQ(first["delivery"]["interval_ms"], 0);
  EXPECT_EQ(first["delivery"]["track_name"], "lkros.data.battery.state");
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  const auto message = makeBatteryState();
  ASSERT_TRUE(
    publishUntil(executor, publisher, message, [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  EXPECT_EQ(session.state->pushed_data_track_frames[0].track_name, track_name);
  EXPECT_EQ(
    deserializeMessage<sensor_msgs::msg::BatteryState>(session.state->pushed_data_track_frames[0].frame.payload),
    message);
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

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

  auto registry = makeLeaseManager(*node, session);

  const auto failed =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 500)}));

  expectStatusEntry(failed, "topic", topic.c_str(), "active");
  EXPECT_EQ(failed["interface_type"], rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>());
  EXPECT_EQ(failed["delivery"]["kind"], "data");
  EXPECT_EQ(failed["delivery"]["interval_ms"], 500);
  EXPECT_TRUE(failed["delivery"]["track_name"].get<std::string>().empty());

  const auto recovered =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 500)}));

  expectStatusEntry(recovered, "topic", topic.c_str(), "active");
  EXPECT_EQ(recovered["interface_type"], rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>());
  EXPECT_EQ(recovered["delivery"]["kind"], "data");
  EXPECT_EQ(recovered["delivery"]["interval_ms"], 500);
  EXPECT_EQ(recovered["delivery"]["track_name"], "lkros.data.battery.failed_publish_response");
  EXPECT_EQ(publish_attempt_count, 2);
}

TEST(SubscriptionLeaseManagerTest, PruneExpiredLeasesKeepsSubscriptionAndRecomputesInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_prune_shared_interval_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/prune_shared_interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session, nullptr, kShortHeartbeatLeaseDuration);

  sendHeartbeat(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 50)}));
  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);

  const auto shared =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(topic, 300)}));
  ASSERT_EQ(shared["delivery"]["interval_ms"], 50);
  const auto unpublished_before_prune = session.state->unpublished_data_track_names;

  registry.pruneExpiredLeases();

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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic_a));
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic_b));

  auto registry = makeLeaseManager(*node, session, nullptr, kShortHeartbeatLeaseDuration);

  sendHeartbeat(
    registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic_a, 0), makeTopicDemand(topic_b, 0)}));

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

  EXPECT_EQ(
    session.state->unpublished_data_track_names, std::vector<std::string>{"lkros.data.battery.omitted_expires"});

  const auto renewed_a =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic_a, 0)}));
  EXPECT_EQ(renewed_a["delivery"]["track_name"], "lkros.data.battery.omitted_stays_alive");
}

TEST(SubscriptionLeaseManagerTest, CreatesVideoSubscriptionsForRosTopicsAndOtherSources)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_test");
  FakeRoomConnection session;
  const video::StreamConfig video_config = makeOtherSourceConfig();
  const std::string video_topic = "/camera/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, video_topic));

  auto registry = makeLeaseManager(*node, session, &video_config);

  const auto topic_status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(video_topic)}));
  const auto source_status = sendHeartbeatAndExtractStatus(
    registry, *session.state, "bob", makeHeartbeat({makeOtherVideoDemand("/sources/front")}));

  EXPECT_EQ(topic_status["delivery"]["kind"], "video");
  EXPECT_EQ(topic_status["delivery"]["track_name"], "lkros.video.camera.front");
  EXPECT_EQ(source_status["delivery"]["kind"], "video");
  EXPECT_FALSE(source_status["delivery"]["track_name"].get<std::string>().empty());
}

TEST(SubscriptionLeaseManagerTest, EquivalentRosVideoRequestsShareCanonicalSubscriptionAndTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_canonical_topic_test");
  FakeRoomConnection session;
  const std::string canonical_topic = "/camera/front/image";
  const std::string variant_topic = "  camera/front/image  ";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(canonical_topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, canonical_topic));

  auto registry = makeLeaseManager(*node, session);

  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(canonical_topic)}));
  const auto second =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(variant_topic)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();

  EXPECT_EQ(first["name"], canonical_topic);
  EXPECT_EQ(second["name"], canonical_topic);
  EXPECT_EQ(first["delivery"]["kind"], "video");
  EXPECT_EQ(second["delivery"]["track_name"], track_name);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));
  ASSERT_TRUE(publishUntil(
    executor, publisher, makeRgbImage(), [&]() { return session.state->publishedVideoTrackCount() == 1U; }));
  EXPECT_EQ(session.state->published_video_track_names, (std::vector<std::string>{track_name}));
}

TEST(SubscriptionLeaseManagerTest, EquivalentOtherVideoRequestsShareCanonicalSubscriptionAndTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_canonical_other_test");
  FakeRoomConnection session;
  const video::StreamConfig video_config = makeOtherSourceConfig();
  const std::string canonical_source = "/sources/front";
  const std::string variant_source = "  /sources/front  ";

  auto registry = makeLeaseManager(*node, session, &video_config);

  const auto first = sendHeartbeatAndExtractStatus(
    registry, *session.state, "alice", makeHeartbeat({makeOtherVideoDemand(canonical_source)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();
  ASSERT_TRUE(waitUntil([&session]() { return session.state->publishedVideoTrackCount() == 1U; }));

  const auto second = sendHeartbeatAndExtractStatus(
    registry, *session.state, "bob", makeHeartbeat({makeOtherVideoDemand(variant_source)}));

  EXPECT_EQ(first["name"], canonical_source);
  EXPECT_EQ(second["name"], canonical_source);
  EXPECT_EQ(first["delivery"]["kind"], "video");
  EXPECT_EQ(second["delivery"]["track_name"], track_name);

  EXPECT_EQ(session.state->published_video_track_names, (std::vector<std::string>{track_name}));
}

TEST(SubscriptionLeaseManagerTest, VideoLeaseExpiryBeforeFirstFrameAllowsLaterRecreation)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_restart_before_first_frame_test");
  FakeRoomConnection session;
  const std::string topic = "/camera/restart_before_first_frame";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session, nullptr, kShortHeartbeatLeaseDuration);
  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);
  registry.pruneExpiredLeases();

  EXPECT_TRUE(session.state->unpublished_video_track_names.empty());
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 0U; }));

  const auto recreated =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  EXPECT_EQ(recreated["delivery"]["track_name"], track_name);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeRgbImage(), [&]() { return session.state->publishedVideoTrackCount() == 1U; }));
  EXPECT_EQ(session.state->published_video_track_names, (std::vector<std::string>{track_name}));
}

TEST(SubscriptionLeaseManagerTest, PerStreamPublishOptionsAreAppliedToEachPublishedVideoTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_publish_config_test");
  FakeRoomConnection session;
  const std::string first_topic = "/camera/publish_config/one";
  const std::string second_topic = "/camera/publish_config/two";
  livekit::TrackPublishOptions first_publish_options;
  first_publish_options.video_codec = static_cast<livekit::VideoCodec>(1);
  first_publish_options.video_encoding = livekit::VideoEncodingOptions{900000, 24.0};
  first_publish_options.simulcast = true;
  livekit::TrackPublishOptions second_publish_options;
  second_publish_options.video_codec = static_cast<livekit::VideoCodec>(3);
  second_publish_options.video_encoding = livekit::VideoEncodingOptions{250000, 12.0};
  second_publish_options.simulcast = false;

  video::StreamConfig config = video::makeDefaultConfig();
  config.ros_topic_rules.push_back(
    {RosResourcePattern::fromCanonical(first_topic), "first_publish_config", "", first_publish_options});
  config.ros_topic_rules.push_back(
    {RosResourcePattern::fromCanonical(second_topic), "second_publish_config", "", second_publish_options});

  auto first_publisher = node->create_publisher<sensor_msgs::msg::Image>(first_topic, rclcpp::QoS(10));
  auto second_publisher = node->create_publisher<sensor_msgs::msg::Image>(second_topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, first_topic));
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, second_topic));

  auto registry = makeLeaseManager(*node, session, &config);
  const auto first_status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(first_topic)}));
  const auto second_status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "bob", makeHeartbeat({makeTopicDemand(second_topic)}));

  ASSERT_TRUE(spinUntil(executor, [&first_publisher]() { return first_publisher->get_subscription_count() == 1U; }));
  ASSERT_TRUE(spinUntil(executor, [&second_publisher]() { return second_publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(
    executor, first_publisher, makeRgbImage(2, 2), [&]() { return session.state->publishedVideoTrackCount() == 1U; }));
  ASSERT_TRUE(publishUntil(
    executor, second_publisher, makeRgbImage(4, 4), [&]() { return session.state->publishedVideoTrackCount() == 2U; }));

  EXPECT_EQ(
    session.state->published_video_track_names,
    (std::vector<std::string>{
      first_status["delivery"]["track_name"].get<std::string>(),
      second_status["delivery"]["track_name"].get<std::string>(),
    }));

  ASSERT_EQ(session.state->published_video_options.size(), 2U);
  const auto & first_options = session.state->published_video_options[0];
  const auto & second_options = session.state->published_video_options[1];
  EXPECT_EQ(first_options.video_codec, first_publish_options.video_codec);
  ASSERT_TRUE(first_options.video_encoding.has_value());
  ASSERT_TRUE(first_publish_options.video_encoding.has_value());
  EXPECT_EQ(first_options.video_encoding->max_bitrate, first_publish_options.video_encoding->max_bitrate);
  EXPECT_DOUBLE_EQ(first_options.video_encoding->max_framerate, first_publish_options.video_encoding->max_framerate);
  EXPECT_EQ(first_options.simulcast, first_publish_options.simulcast);
  EXPECT_EQ(second_options.video_codec, second_publish_options.video_codec);
  ASSERT_TRUE(second_options.video_encoding.has_value());
  ASSERT_TRUE(second_publish_options.video_encoding.has_value());
  EXPECT_EQ(second_options.video_encoding->max_bitrate, second_publish_options.video_encoding->max_bitrate);
  EXPECT_DOUBLE_EQ(second_options.video_encoding->max_framerate, second_publish_options.video_encoding->max_framerate);
  EXPECT_EQ(second_options.simulcast, second_publish_options.simulcast);
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
  const auto first =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 1000)}));
  const std::string track_name = first["delivery"]["track_name"].get<std::string>();

  registry.onRemoteParticipantDisconnected("alice");
  const auto renewed =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 1000)}));

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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session, nullptr, kShortHeartbeatLeaseDuration);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);
  registry.pruneExpiredLeases();

  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
}

TEST(SubscriptionLeaseManagerTest, PruneExpiredLeasesUnpublishesPublishedVideoTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_prune_video_unpublish_test");
  FakeRoomConnection session;
  const std::string topic = "/camera/prune_expired";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::Image>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session, nullptr, kShortHeartbeatLeaseDuration);
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = status["delivery"]["track_name"].get<std::string>();

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeRgbImage(), [&]() { return session.state->publishedVideoTrackCount() == 1U; }));
  EXPECT_EQ(session.state->published_video_track_names, std::vector<std::string>{track_name});

  std::this_thread::sleep_for(kShortHeartbeatLeaseDuration + kLeaseWaitBuffer);
  registry.pruneExpiredLeases();

  EXPECT_EQ(session.state->unpublished_video_track_names, std::vector<std::string>{track_name});
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto push_entered = std::make_shared<std::promise<void>>();
  auto push_entered_future = push_entered->get_future();
  auto release_push = std::make_shared<std::promise<void>>();
  auto release_push_future = release_push->get_future().share();
  std::atomic<int> push_call_count{0};
  session.state->try_push_data_track_handler = [push_entered, release_push_future, &push_call_count](
                                                 const std::string &, const livekit::DataTrackFrame &) {
    const int call_number = push_call_count.fetch_add(1) + 1;
    if (call_number == 1) {
      push_entered->set_value();
      release_push_future.wait();
    }
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
  };

  auto registry = makeLeaseManager(*node, session);
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
  EXPECT_EQ(push_call_count.load(), 1);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionLeaseManagerTest, ShutdownAllowsFreshManagerToRecreateSameDataTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_reuse_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/shutdown_reuse";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
  const auto active =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = active["delivery"]["track_name"].get<std::string>();

  registry.shutdown();

  auto reopened = makeLeaseManager(*node, session);
  const auto reopened_status =
    sendHeartbeatAndExtractStatus(reopened, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  EXPECT_EQ(reopened_status["delivery"]["track_name"], track_name);
  EXPECT_EQ(session.state->published_data_track_names, (std::vector<std::string>{track_name, track_name}));
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
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  std::atomic<int> push_attempt_count{0};
  session.state->try_push_data_track_handler = [&push_attempt_count](
                                                 const std::string &, const livekit::DataTrackFrame &) {
    push_attempt_count.fetch_add(1);
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::failure(
      livekit::LocalDataTrackTryPushError{livekit::LocalDataTrackTryPushErrorCode::QUEUE_FULL, "queue full"});
  };

  auto registry = makeLeaseManager(*node, session);
  sendHeartbeat(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return push_attempt_count.load() >= 1; }));
  const auto status =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  expectStatusEntry(status, "topic", topic.c_str(), "active");
}

TEST(SubscriptionLeaseManagerTest, OnRemoteParticipantDisconnectedRejectsEmptyIdentity)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_requester_test");
  FakeRoomConnection session;
  auto registry = makeLeaseManager(*node, session);

  expectInvalidArgumentMessage(
    [&registry]() { registry.onRemoteParticipantDisconnected(""); }, "requester_identity is required");
}

TEST(SubscriptionLeaseManagerTest, ShutdownReportsNotFoundSubscriptionsAndFurtherLifecycleCallsAreNoOps)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_rejection_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/shutdown_rejection";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopic<sensor_msgs::msg::BatteryState>(executor, node, topic));

  auto registry = makeLeaseManager(*node, session);
  const auto active =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  const std::string track_name = active["delivery"]["track_name"].get<std::string>();

  registry.shutdown();

  const auto unavailable =
    sendHeartbeatAndExtractStatus(registry, *session.state, "alice", makeHeartbeat({makeTopicDemand(topic, 0)}));
  expectStatusEntry(unavailable, "topic", topic.c_str(), "error");
  EXPECT_EQ(unavailable["error"]["reason"], "not_found");
  EXPECT_EQ(unavailable["error"]["message"], "Subscription registry is shut down.");

  registry.pruneExpiredLeases();

  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, ForbiddenTopicReturnsError)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});

  auto manager = makeManager(deny_all);

  sendHeartbeat(manager, *state_, "requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, OtherVideoBypassesRosAccessPolicyAndReturnsVideoStatus)
{
  const AccessPolicy deny_all = makeSubscribePolicy({}, {"*"});
  const video::StreamConfig video_config = makeOtherSourceConfig();

  auto manager = makeManager(deny_all, &video_config);

  sendHeartbeat(manager, *state_, "requester-1", makeHeartbeat({makeOtherVideoDemand("/sources/front")}));

  const auto status = extractPublishedStatusEntry(*state_, "requester-1");
  expectStatusEntry(status, "other_video", "/sources/front", "active");
  const auto & delivery = status["delivery"];
  EXPECT_EQ(delivery["kind"], "video");
  EXPECT_FALSE(delivery["track_name"].get<std::string>().empty());
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, MissingOtherVideoReturnsErrorOnSourceIdField)
{
  const video::StreamConfig video_config = makeOtherSourceConfig();

  auto manager = makeManager(access_policy_, &video_config);

  sendHeartbeat(manager, *state_, "requester-1", makeHeartbeat({makeOtherVideoDemand("/sources/missing")}));

  expectPublishedError(
    *state_,
    "requester-1",
    "other_video",
    "/sources/missing",
    "not_found",
    "Unknown other video source '/sources/missing'.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, ActiveSubscriptionPublishesSubscriptionStatusEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state");

  auto manager = makeManager(access_policy_);

  sendHeartbeat(
    manager, *state_, "requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["v"], protocol::kProtocolVersion);
  EXPECT_EQ(envelope["type"], protocol::kStatusTopic);
  EXPECT_EQ(envelope["session_id"], "session-1");
  ASSERT_TRUE(envelope["lease_expires_in_ms"].is_number_integer());
  EXPECT_GT(envelope["lease_expires_in_ms"].get<std::int64_t>(), 0);

  const auto status = extractStatusEntry(envelope);
  const auto & delivery = status["delivery"];
  expectStatusEntry(status, "topic", "/battery_state", "active");
  EXPECT_EQ(status["interface_type"], rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>());
  EXPECT_EQ(delivery["track_name"], "lkros.data.battery_state");
  EXPECT_EQ(delivery["interval_ms"], 100);
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, OmittedHeartbeatTargetRemainsActiveUntilItsOriginalLeaseExpires)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto battery_a = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_a");
  auto battery_b = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_b");

  auto manager = makeManager(access_policy_);

  sendHeartbeat(
    manager,
    *state_,
    "requester-1",
    makeHeartbeat({makeTopicDemand("/battery_a", 100), makeTopicDemand("/battery_b", 200)}));
  sendHeartbeat(manager, *state_, "requester-1", makeHeartbeat({makeTopicDemand("/battery_a", 100)}));

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  ASSERT_TRUE(envelope.contains("subscriptions"));
  ASSERT_EQ(envelope["subscriptions"].size(), 1U);

  const auto battery_a_status = findStatusEntry(envelope, "topic", "/battery_a");
  ASSERT_TRUE(battery_a_status.has_value());
  expectStatusEntry(*battery_a_status, "topic", "/battery_a", "active");
  EXPECT_FALSE(findStatusEntry(envelope, "topic", "/battery_b").has_value());

  EXPECT_TRUE(state_->unpublished_data_track_names.empty());

  (void)battery_a;
  (void)battery_b;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, AnonymousHeartbeatRenewsKnownClientSession)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state");

  auto manager = makeManager(access_policy_);

  const auto heartbeat = makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1"));
  sendHeartbeat(manager, *state_, "requester-1", heartbeat);
  sendHeartbeat(manager, *state_, "", heartbeat);

  const auto envelope = extractPublishedStatusEnvelope(*state_, "requester-1");
  EXPECT_EQ(envelope["session_id"], "session-1");

  const auto status = extractStatusEntry(envelope);
  expectStatusEntry(status, "topic", "/battery_state", "active");
  (void)publisher;
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, AnonymousHeartbeatWithoutResolvableClientSessionIsDropped)
{
  auto manager = makeManager(access_policy_);

  sendHeartbeat(
    manager, *state_, "", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("unknown-session")));
  sendHeartbeat(manager, *state_, "", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  EXPECT_EQ(state_->publish_data_call_count, 0);
}

TEST_F(
  SubscriptionLeaseManagerHeartbeatTest,
  EmptyHeartbeatBindsClientSessionLeaseThatAnonymousHeartbeatCanStillUseAfterConflict)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state");

  auto manager = makeManager(access_policy_);

  const auto bind_heartbeat = makeHeartbeat({}, std::string("session-1"));
  sendHeartbeat(manager, *state_, "requester-1", bind_heartbeat);
  sendHeartbeat(manager, *state_, "requester-2", bind_heartbeat);

  EXPECT_EQ(state_->publish_data_call_count, 0);

  sendHeartbeat(
    manager, *state_, "", makeHeartbeat({makeTopicDemand("/battery_state", 100)}, std::string("session-1")));

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

  sendHeartbeat(manager, *state_, "requester-1", makeHeartbeat({makeTopicDemand("/battery_state", 100)}));

  expectPublishedError(
    *state_, "requester-1", "topic", "/battery_state", "forbidden", "ROS topic '/battery_state' not permitted.");
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, PublishControlPacketFailureIsHandledGracefully)
{
  state_->throw_on_publish_data = true;

  auto manager = makeManager(access_policy_);

  EXPECT_NO_THROW(manager.handleHeartbeatPayload(
    "requester-1", heartbeatPayloadBytes(makeHeartbeat({makeTopicDemand("/nonexistent_topic", 100)}))));
  EXPECT_EQ(state_->publish_data_call_count, 1);
}

TEST_F(SubscriptionLeaseManagerHeartbeatTest, MixedSubscriptionResultsArePublishedInOneEnvelope)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto publisher = advertiseTopic<sensor_msgs::msg::BatteryState>(executor, node_, "/battery_state");

  auto manager = makeManager(access_policy_);

  sendHeartbeat(
    manager,
    *state_,
    "requester-1",
    makeHeartbeat({makeTopicDemand("/battery_state", 100), makeTopicDemand("/nonexistent_topic", 100)}));

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
