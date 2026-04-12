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
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_registry.hpp"
#include "video_stream_manager.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitForTopicType;
using test_support::waitUntil;

const auto kFarFuture = std::chrono::steady_clock::now() + std::chrono::hours(1);

struct CdrFrame
{
  std::string track_name;
  std::vector<std::uint8_t> data;
};

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
std::vector<std::uint8_t> serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
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

VideoConfig makeConfiguredVideoConfig()
{
  VideoConfig config = makeDefaultVideoConfig();
  ConfiguredVideoPipeline configured_pipeline;
  configured_pipeline.ingress_fragment = "videotestsrc is-live=true pattern=black";
  config.configured_sources.emplace("/sources/front", std::move(configured_pipeline));
  return config;
}

SendCdrMessageFn noopCdrSend()
{
  return [](const std::string &, const std::uint8_t *, std::size_t) {};
}

PublishCdrTrackFn noopCdrPublish()
{
  return [](const std::string &, std::size_t) {};
}

UnpublishCdrTrackFn noopCdrUnpublish()
{
  return [](const std::string &) {};
}

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

TEST(SubscriptionRegistryTest, RenewSubscriptionReturnsDataTrackForNonVideoTopics)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_data_track_test");
  const std::string topic = "/battery/state";
  const std::string requested_topic = "  //battery/state/  ";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_track_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_track_names](const std::string & name, std::size_t) { published_track_names.push_back(name); },
    noopCdrUnpublish(),
    nullptr);

  const auto response = registry.renewSubscription("alice", requested_topic, 0, kFarFuture);
  const auto second_response = registry.renewSubscription("bob", topic, 0, kFarFuture);

  EXPECT_EQ(response.target.name, topic);
  EXPECT_EQ(response.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kData);
  ASSERT_EQ(published_track_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_track_names[0]);
  EXPECT_EQ(second_response.target.name, topic);
  EXPECT_EQ(second_response.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(second_response.delivery_kind, StreamDeliveryKind::kData);
  EXPECT_EQ(second_response.track_name, published_track_names[0]);
}

TEST(SubscriptionRegistryTest, RenewSubscriptionNormalizesRawHeartbeatTopicSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_raw_heartbeat_topic_test");
  const std::string topic = "/battery/raw_heartbeat_topic";
  const SubscriptionRequest raw_subscription{
    {SubscriptionTargetKind::Topic, "  //battery//raw_heartbeat_topic/  "}, std::nullopt};
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_track_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_track_names](const std::string & name, std::size_t) { published_track_names.push_back(name); },
    noopCdrUnpublish(),
    nullptr);

  const auto raw_response = registry.renewSubscription("alice", raw_subscription, kFarFuture);
  const auto canonical_response = registry.renewSubscription(
    "bob", SubscriptionRequest{{SubscriptionTargetKind::Topic, topic}, std::nullopt}, kFarFuture);

  EXPECT_EQ(raw_response.target.name, topic);
  EXPECT_EQ(raw_response.delivery_kind, StreamDeliveryKind::kData);
  EXPECT_EQ(canonical_response.target.name, topic);
  ASSERT_EQ(published_track_names.size(), 1U);
  EXPECT_EQ(raw_response.track_name, published_track_names[0]);
  EXPECT_EQ(canonical_response.track_name, published_track_names[0]);
  EXPECT_TRUE(registry.hasSubscription(topic));
}

TEST(SubscriptionRegistryTest, SendsRawCdrFramesOnGenericSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_send_test");
  const std::string topic = "/battery/send";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<CdrFrame> cdr_frames;
  SubscriptionRegistry registry(
    *node,
    [&cdr_frames](const std::string & name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({name, std::vector<std::uint8_t>(data, data + size)});
    },
    noopCdrPublish(),
    noopCdrUnpublish(),
    nullptr);

  registry.renewSubscription("alice", topic, 0, kFarFuture);

  const auto message = makeBatteryState();
  ASSERT_TRUE(publishUntil(executor, publisher, message, [&]() { return cdr_frames.size() == 1U; }));

  ASSERT_EQ(cdr_frames.size(), 1U);
  EXPECT_EQ(cdr_frames[0].track_name, "ros.data.battery.send");
  const auto decoded = deserializeMessage<sensor_msgs::msg::BatteryState>(cdr_frames[0].data);
  EXPECT_EQ(decoded, message);
}

TEST(SubscriptionRegistryTest, BestEffortPublisherDeliversToDataSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_best_effort_data_test");
  const std::string topic = "/battery/best_effort";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10).best_effort());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<CdrFrame> cdr_frames;
  SubscriptionRegistry registry(
    *node,
    [&cdr_frames](const std::string & name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({name, std::vector<std::uint8_t>(data, data + size)});
    },
    noopCdrPublish(),
    noopCdrUnpublish(),
    nullptr);

  registry.renewSubscription("alice", topic, 0, kFarFuture);

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));
  EXPECT_EQ(cdr_frames[0].track_name, "ros.data.battery.best_effort");
}

TEST(SubscriptionRegistryTest, AppliesMinimumRequesterInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_interval_test");
  const std::string topic = "/battery/interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<CdrFrame> cdr_frames;
  SubscriptionRegistry registry(
    *node,
    [&cdr_frames](const std::string & name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({name, std::vector<std::uint8_t>(data, data + size)});
    },
    noopCdrPublish(),
    noopCdrUnpublish(),
    nullptr);

  registry.renewSubscription("alice", topic, 300, kFarFuture);
  const auto response = registry.renewSubscription("bob", topic, 150, kFarFuture);
  ASSERT_EQ(response.applied_interval_ms, 150);

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));

  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(cdr_frames.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 2U; }));
}

TEST(SubscriptionRegistryTest, ClampsNegativeRequesterIntervalToZero)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_negative_interval_test");
  const std::string topic = "/battery/negative_interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  const auto first_response = registry.renewSubscription("alice", topic, -25, kFarFuture);
  const auto second_response = registry.renewSubscription("bob", topic, 150, kFarFuture);

  EXPECT_EQ(first_response.applied_interval_ms, 0);
  EXPECT_EQ(second_response.applied_interval_ms, 0);
}

TEST(SubscriptionRegistryTest, RenewSubscriptionCreatesVideoSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const std::string topic = "/camera/front";
  const std::string requested_topic = "  camera/front  ";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &video_stream_manager);

  const auto response = registry.renewSubscription("alice", requested_topic, 0, kFarFuture);
  const auto second_response = registry.renewSubscription("bob", topic, 0, kFarFuture);

  EXPECT_EQ(response.target.name, topic);
  EXPECT_EQ(response.interface_type, "sensor_msgs/msg/Image");
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_EQ(response.track_name, "ros.video.camera.front");
  EXPECT_EQ(second_response.track_name, "ros.video.camera.front");
}

TEST(SubscriptionRegistryTest, RenewSubscriptionCreatesConfiguredSourceSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_configured_source_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const VideoConfig video_config = makeConfiguredVideoConfig();

  SubscriptionRegistry registry(
    *node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &video_stream_manager, &video_config);

  const auto response = registry.renewSubscription(
    "alice",
    SubscriptionRequest{{SubscriptionTargetKind::ConfiguredSource, "/sources/front"}, std::nullopt},
    kFarFuture);

  EXPECT_EQ(response.target.kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(response.target.name, "/sources/front");
  EXPECT_EQ(response.interface_type, "");
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_EQ(response.track_name, "ros.video.configured_source.%2Fsources%2Ffront");
  EXPECT_TRUE(registry.hasSubscription("/sources/front", SubscriptionTargetKind::ConfiguredSource));
}

TEST(SubscriptionRegistryTest, RenewSubscriptionTrimsRawConfiguredSourceHeartbeatSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_raw_configured_source_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const VideoConfig video_config = makeConfiguredVideoConfig();
  const std::string configured_source_name = "/sources/front";
  const SubscriptionRequest raw_subscription{
    {SubscriptionTargetKind::ConfiguredSource, "  /sources/front  "}, std::nullopt};
  const SubscriptionRequest canonical_subscription{
    {SubscriptionTargetKind::ConfiguredSource, configured_source_name}, std::nullopt};

  SubscriptionRegistry registry(
    *node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &video_stream_manager, &video_config);

  const auto raw_response = registry.renewSubscription("alice", raw_subscription, kFarFuture);
  const auto canonical_response = registry.renewSubscription("bob", canonical_subscription, kFarFuture);

  EXPECT_EQ(raw_response.target.name, configured_source_name);
  EXPECT_EQ(canonical_response.target.name, configured_source_name);
  EXPECT_EQ(raw_response.track_name, canonical_response.track_name);
  EXPECT_TRUE(registry.hasSubscription(configured_source_name, SubscriptionTargetKind::ConfiguredSource));

  registry.revokeRequesterLeases("bob");
  EXPECT_TRUE(registry.hasSubscription(configured_source_name, SubscriptionTargetKind::ConfiguredSource));

  registry.revokeRequesterLeases("alice");
  EXPECT_FALSE(registry.hasSubscription(configured_source_name, SubscriptionTargetKind::ConfiguredSource));
}

TEST(SubscriptionRegistryTest, TopicAndConfiguredSourceStayDistinctWhenNamesMatch)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_distinct_target_names_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const std::string shared_name = "/sources/front";
  const VideoConfig video_config = makeConfiguredVideoConfig();
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(shared_name, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, shared_name, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(
    *node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &video_stream_manager, &video_config);

  const auto topic_response = registry.renewSubscription("alice", shared_name, 0, kFarFuture);
  const auto source_response = registry.renewSubscription(
    "bob", SubscriptionRequest{{SubscriptionTargetKind::ConfiguredSource, shared_name}, std::nullopt}, kFarFuture);

  EXPECT_EQ(topic_response.target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(topic_response.delivery_kind, StreamDeliveryKind::kData);
  EXPECT_EQ(source_response.target.kind, SubscriptionTargetKind::ConfiguredSource);
  EXPECT_EQ(source_response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_TRUE(registry.hasSubscription(shared_name, SubscriptionTargetKind::Topic));
  EXPECT_TRUE(registry.hasSubscription(shared_name, SubscriptionTargetKind::ConfiguredSource));
}

TEST(SubscriptionRegistryTest, ThrowsUnavailableWhenNoVideoStreamManager)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_unavailable_test");
  const std::string topic = "/camera/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);
  EXPECT_THROW(registry.renewSubscription("alice", topic, 0, kFarFuture), StreamUnavailableError);
}

TEST(SubscriptionRegistryTest, RevokeRequesterLeasesPreservesSharedSubscriptionsOwnedByOthers)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_remove_requester_shared_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const std::string alice_only_topic = "/battery/alice_only";
  const std::string shared_data_topic = "/battery/shared";
  const std::string shared_video_topic = "/camera/shared";
  auto alice_only_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(alice_only_topic, rclcpp::QoS(10));
  auto shared_data_publisher =
    node->create_publisher<sensor_msgs::msg::BatteryState>(shared_data_topic, rclcpp::QoS(10));
  auto shared_video_publisher = node->create_publisher<sensor_msgs::msg::Image>(shared_video_topic, rclcpp::QoS(10));
  (void)alice_only_publisher;
  (void)shared_data_publisher;
  (void)shared_video_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, alice_only_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, shared_data_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, shared_video_topic, "sensor_msgs/msg/Image"));

  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    noopCdrPublish(),
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    &video_stream_manager);

  const auto alice_only = registry.renewSubscription("alice", alice_only_topic, 50, kFarFuture);
  const auto shared_data = registry.renewSubscription("alice", shared_data_topic, 50, kFarFuture);
  registry.renewSubscription("bob", shared_data_topic, 250, kFarFuture);
  const auto shared_video = registry.renewSubscription("alice", shared_video_topic, 0, kFarFuture);
  registry.renewSubscription("bob", shared_video_topic, 0, kFarFuture);
  ASSERT_TRUE(registry.onCdrTrackPublished(alice_only.track_name, 0));
  ASSERT_TRUE(registry.onCdrTrackPublished(shared_data.track_name, 0));

  registry.revokeRequesterLeases("alice");

  EXPECT_FALSE(registry.hasSubscription(alice_only_topic));
  EXPECT_TRUE(registry.hasSubscription(shared_data_topic));
  EXPECT_TRUE(registry.hasSubscription(shared_video_topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], alice_only.track_name);

  const auto shared_data_after_disconnect = registry.renewSubscription("bob", shared_data_topic, 250, kFarFuture);
  EXPECT_EQ(shared_data_after_disconnect.track_name, shared_data.track_name);
  EXPECT_EQ(shared_data_after_disconnect.applied_interval_ms, 250);

  const auto shared_video_after_disconnect = registry.renewSubscription("bob", shared_video_topic, 0, kFarFuture);
  EXPECT_EQ(shared_video_after_disconnect.track_name, shared_video.track_name);
}

TEST(SubscriptionRegistryTest, ParticipantRefreshReplaysPublishedCdrTrackWithoutDroppingLease)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_refresh_replay_test");
  const std::string topic = "/battery/refresh_replay";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto response = registry.renewSubscription("alice", topic, 1000, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  EXPECT_TRUE(registry.onCdrTrackPublished(response.track_name, 0));

  registry.markRequesterForCdrReplay("alice", 0);
  registry.replayCdrTracksForRequester("alice");

  EXPECT_TRUE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], response.track_name);
  ASSERT_EQ(published_names.size(), 2U);
  EXPECT_EQ(published_names[0], published_names[1]);
}

TEST(SubscriptionRegistryTest, NewRequesterReplaysAlreadyPublishedCdrTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_new_requester_replay_test");
  const std::string topic = "/battery/new_requester_replay";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto first_response = registry.renewSubscription("alice", topic, 1000, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  EXPECT_TRUE(registry.onCdrTrackPublished(first_response.track_name, 0));

  const auto second_response = registry.renewSubscription("bob", topic, 250, kFarFuture);
  EXPECT_EQ(second_response.track_name, first_response.track_name);

  registry.replayCdrTracksForRequester("bob");

  EXPECT_TRUE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], first_response.track_name);
  ASSERT_EQ(published_names.size(), 2U);
  EXPECT_EQ(published_names[0], published_names[1]);
}

TEST(SubscriptionRegistryTest, DisconnectAndExpiryPrunePathsRecomputeSurvivingDataIntervalTheSameWay)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_prune_interval_recompute_test");
  const std::string disconnect_topic = "/battery/prune_disconnect";
  const std::string expiry_topic = "/battery/prune_expiry";
  auto disconnect_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(disconnect_topic, rclcpp::QoS(10));
  auto expiry_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(expiry_topic, rclcpp::QoS(10));
  (void)disconnect_publisher;
  (void)expiry_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, disconnect_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, expiry_topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto disconnect_initial = registry.renewSubscription("alice", disconnect_topic, 50, kFarFuture);
  registry.renewSubscription("bob", disconnect_topic, 250, kFarFuture);
  const auto expiry_initial = registry.renewSubscription("carol", expiry_topic, 50, past);
  registry.renewSubscription("dave", expiry_topic, 250, kFarFuture);

  registry.revokeRequesterLeases("alice");
  const auto disconnect_after_prune = registry.renewSubscription("bob", disconnect_topic, 250, kFarFuture);
  EXPECT_EQ(disconnect_after_prune.track_name, disconnect_initial.track_name);
  EXPECT_EQ(disconnect_after_prune.applied_interval_ms, 250);

  registry.pruneExpiredLeases();
  const auto expiry_after_prune = registry.renewSubscription("dave", expiry_topic, 250, kFarFuture);
  EXPECT_EQ(expiry_after_prune.track_name, expiry_initial.track_name);
  EXPECT_EQ(expiry_after_prune.applied_interval_ms, 250);
}

TEST(SubscriptionRegistryTest, ResetSessionStateClearsDataAndVideoSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_reset_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const std::string data_topic = "/battery/state";
  const std::string video_topic = "/camera/front";
  auto data_pub = node->create_publisher<sensor_msgs::msg::BatteryState>(data_topic, rclcpp::QoS(10));
  auto video_pub = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)data_pub;
  (void)video_pub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, data_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, video_topic, "sensor_msgs/msg/Image"));

  std::vector<std::string> published_track_names;
  std::vector<std::string> unpublished_track_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_track_names](const std::string & name, std::size_t) { published_track_names.push_back(name); },
    [&unpublished_track_names](const std::string & name) { unpublished_track_names.push_back(name); },
    &video_stream_manager);

  const auto response = registry.renewSubscription("alice", data_topic, 0, kFarFuture);
  registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  ASSERT_EQ(published_track_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_track_names[0]);
  EXPECT_TRUE(registry.onCdrTrackPublished(published_track_names[0], 0));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(
    executor, video_pub, image, [&]() { return session.state->published_video_track_names.size() == 1U; }));

  registry.resetSessionState();

  EXPECT_FALSE(registry.hasSubscription(data_topic));
  EXPECT_FALSE(registry.hasSubscription(video_topic));
  ASSERT_EQ(unpublished_track_names.size(), 1U);
  EXPECT_EQ(unpublished_track_names[0], published_track_names[0]);
  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], "ros.video.camera.front");
}

TEST(SubscriptionRegistryTest, UnpublishesDataTrackWhenLastRequesterExpires)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_cdr_unpublish_test");
  const std::string topic = "/battery/cdr_expire";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto response = registry.renewSubscription("alice", topic, 0, past);
  ASSERT_EQ(published_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_names[0]);
  registry.onCdrTrackPublished(published_names[0], 0);

  registry.pruneExpiredLeases();

  EXPECT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], published_names[0]);
}

TEST(SubscriptionRegistryTest, PruneExpiredLeasesDoesNotUnpublishPendingCdrTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_cdr_pending_sweep_test");
  const std::string topic = "/battery/cdr_pending_sweep";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  registry.renewSubscription("alice", topic, 0, past);
  ASSERT_EQ(published_names.size(), 1U);

  registry.pruneExpiredLeases();

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_TRUE(unpublished_names.empty());
}

TEST(SubscriptionRegistryTest, OnCdrTrackPublishedReturnsFalseForUnknownTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_cdr_unknown_track_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);
  EXPECT_FALSE(registry.onCdrTrackPublished("ros.data.no.such.topic", 0));
}

TEST(SubscriptionRegistryTest, HasSubscriptionReturnsFalseForWhitespaceTopic)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_whitespace_topic_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  EXPECT_FALSE(registry.hasSubscription("   "));
  EXPECT_FALSE(registry.hasSubscription(""));
}

TEST(SubscriptionRegistryTest, CdrDeliveryLogsAndContinuesOnUnexpectedError)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_send_error_test");
  const std::string topic = "/battery/send_error";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  int send_call_count = 0;
  SubscriptionRegistry registry(
    *node,
    [&send_call_count](const std::string &, const std::uint8_t *, std::size_t) {
      ++send_call_count;
      throw std::runtime_error("unexpected send error");
    },
    noopCdrPublish(),
    noopCdrUnpublish(),
    nullptr);

  registry.renewSubscription("alice", topic, 0, kFarFuture);

  publisher->publish(makeBatteryState());
  ASSERT_TRUE(spinUntil(executor, [&send_call_count]() { return send_call_count == 1; }));
  publisher->publish(makeBatteryState());
  ASSERT_TRUE(spinUntil(executor, [&send_call_count]() { return send_call_count == 2; }));
  EXPECT_EQ(send_call_count, 2);
  EXPECT_TRUE(registry.hasSubscription(topic));
}

TEST(SubscriptionRegistryTest, RenewSubscriptionThrowsOnEmptyRequesterIdentity)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_requester_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  expectInvalidArgumentMessage(
    [&registry]() { (void)registry.renewSubscription("", "/some/topic", 0, kFarFuture); },
    "requester_identity is required");
}

TEST(SubscriptionRegistryTest, RequesterSpecificMethodsThrowOnEmptyRequesterIdentity)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_requester_methods_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  expectInvalidArgumentMessage(
    [&registry]() { registry.markRequesterForCdrReplay("", 0); }, "requester_identity is required");
  expectInvalidArgumentMessage(
    [&registry]() { registry.replayCdrTracksForRequester(""); }, "requester_identity is required");
  expectInvalidArgumentMessage([&registry]() { registry.revokeRequesterLeases(""); }, "requester_identity is required");
}

TEST(SubscriptionRegistryTest, RenewSubscriptionThrowsOnInvalidTopic)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_invalid_topic_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  EXPECT_THROW(registry.renewSubscription("alice", "   ", 0, kFarFuture), std::invalid_argument);
  EXPECT_THROW(registry.renewSubscription("alice", "", 0, kFarFuture), std::invalid_argument);
}

TEST(SubscriptionRegistryTest, RenewSubscriptionRejectsHeartbeatEntriesThatNormalizeToEmptyTargets)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_heartbeat_target_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const VideoConfig video_config = makeConfiguredVideoConfig();

  SubscriptionRegistry registry(
    *node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &video_stream_manager, &video_config);

  const auto expect_invalid_argument = [&](const SubscriptionRequest & subscription, const char * expected_message) {
    expectInvalidArgumentMessage(
      [&registry, &subscription]() { (void)registry.renewSubscription("alice", subscription, kFarFuture); },
      expected_message);
  };

  expect_invalid_argument(
    SubscriptionRequest{{SubscriptionTargetKind::Topic, "   "}, std::nullopt},
    "heartbeat subscription target name must normalize to a non-empty topic name");
  expect_invalid_argument(
    SubscriptionRequest{{SubscriptionTargetKind::ConfiguredSource, "  \t\n  "}, std::nullopt},
    "heartbeat subscription target name must trim to a non-empty configured_source name");
}

TEST(SubscriptionRegistryTest, ShutdownClearsVideoSubscriptionsAndUnpublishesPublishedDataTracks)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_test");
  FakeRoomSession session;
  VideoStreamManager video_stream_manager(*node, session);
  const std::string published_topic = "/battery/shutdown_published";
  const std::string pending_topic = "/battery/shutdown_pending";
  const std::string video_topic = "/camera/shutdown_video";
  auto pub1 = node->create_publisher<sensor_msgs::msg::BatteryState>(published_topic, rclcpp::QoS(10));
  auto pub2 = node->create_publisher<sensor_msgs::msg::BatteryState>(pending_topic, rclcpp::QoS(10));
  auto pub3 = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)pub1;
  (void)pub2;
  (void)pub3;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, published_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, pending_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, video_topic, "sensor_msgs/msg/Image"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    &video_stream_manager);

  registry.renewSubscription("alice", published_topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  registry.onCdrTrackPublished(published_names[0], 0);

  registry.renewSubscription("alice", pending_topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 2U);
  registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  ASSERT_TRUE(publishUntil(
    executor, pub3, makeRgbImage(), [&]() { return session.state->published_video_track_names.size() == 1U; }));

  registry.shutdown();

  EXPECT_FALSE(registry.hasSubscription(published_topic));
  EXPECT_FALSE(registry.hasSubscription(pending_topic));
  EXPECT_FALSE(registry.hasSubscription(video_topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], published_names[0]);
  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], "ros.video.camera.shutdown_video");
}

TEST(SubscriptionRegistryTest, ShutdownPreventsLeaseRecreation)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_terminal_test");
  const std::string topic = "/battery/shutdown_terminal";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    noopCdrUnpublish(),
    nullptr);

  registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  registry.shutdown();

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_THROW(registry.renewSubscription("alice", topic, 0, kFarFuture), StreamUnavailableError);
  EXPECT_EQ(published_names.size(), 1U);
}

TEST(SubscriptionRegistryTest, ShutdownWaitsForActiveSerializedMessageCallback)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_quiesce_test");
  const std::string topic = "/battery/shutdown_quiesce";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  auto send_entered = std::make_shared<std::promise<void>>();
  auto send_entered_future = send_entered->get_future();
  auto release_send = std::make_shared<std::promise<void>>();
  auto release_send_future = release_send->get_future().share();
  std::atomic<int> send_call_count{0};
  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  std::atomic<bool> unpublish_called{false};
  SubscriptionRegistry registry(
    *node,
    [send_entered, release_send_future, &send_call_count](const std::string &, const std::uint8_t *, std::size_t) {
      const int call_number = send_call_count.fetch_add(1) + 1;
      if (call_number == 1) {
        send_entered->set_value();
        release_send_future.wait();
      }
    },
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names, &unpublish_called](const std::string & name) {
      unpublished_names.push_back(name);
      unpublish_called.store(true);
    },
    nullptr);

  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_names[0]);
  EXPECT_TRUE(registry.onCdrTrackPublished(published_names[0], 0));

  std::thread spin_thread([&executor]() { executor.spin(); });

  publisher->publish(makeBatteryState());
  EXPECT_EQ(send_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto shutdown_future = std::async(std::launch::async, [&registry]() { registry.shutdown(); });
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  EXPECT_FALSE(unpublish_called.load());

  release_send->set_value();

  EXPECT_EQ(shutdown_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  shutdown_future.get();
  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_EQ(send_call_count.load(), 1);
  EXPECT_EQ(unpublished_names.size(), 1U);
  if (unpublished_names.size() == 1U) {
    EXPECT_EQ(unpublished_names[0], published_names[0]);
  }

  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionRegistryTest, ResetSessionStateDrainsInFlightSerializedMessageCallbacks)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_reset_quiesce_test");
  const std::string topic = "/battery/reset_quiesce";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  auto send_entered = std::make_shared<std::promise<void>>();
  auto send_entered_future = send_entered->get_future();
  auto release_send = std::make_shared<std::promise<void>>();
  auto release_send_future = release_send->get_future().share();
  std::atomic<int> send_call_count{0};
  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  std::atomic<bool> unpublish_called{false};
  SubscriptionRegistry registry(
    *node,
    [send_entered, release_send_future, &send_call_count](const std::string &, const std::uint8_t *, std::size_t) {
      const int call_number = send_call_count.fetch_add(1) + 1;
      if (call_number == 1) {
        send_entered->set_value();
        release_send_future.wait();
      }
    },
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names, &unpublish_called](const std::string & name) {
      unpublished_names.push_back(name);
      unpublish_called.store(true);
    },
    nullptr);

  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_names[0]);
  EXPECT_TRUE(registry.onCdrTrackPublished(published_names[0], 0));

  std::thread spin_thread([&executor]() { executor.spin(); });

  publisher->publish(makeBatteryState());
  EXPECT_EQ(send_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  publisher->publish(makeBatteryState());

  auto reset_future = std::async(std::launch::async, [&registry]() { registry.resetSessionState(); });
  EXPECT_EQ(reset_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  EXPECT_FALSE(unpublish_called.load());

  release_send->set_value();

  EXPECT_EQ(reset_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  reset_future.get();
  EXPECT_FALSE(registry.hasSubscription(topic));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(send_call_count.load(), 1);
  EXPECT_TRUE(unpublish_called.load());
  EXPECT_EQ(unpublished_names.size(), 1U);
  if (unpublished_names.size() == 1U) {
    EXPECT_EQ(unpublished_names[0], published_names[0]);
  }

  const auto next_response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  EXPECT_EQ(published_names.size(), 2U);
  if (published_names.size() == 2U) {
    EXPECT_EQ(next_response.track_name, published_names[1]);
    EXPECT_TRUE(registry.onCdrTrackPublished(published_names[1], registry.registryGeneration()));
  }
  publisher->publish(makeBatteryState());
  EXPECT_TRUE(waitUntil([&send_call_count]() { return send_call_count.load() == 2; }));

  registry.shutdown();
  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionRegistryTest, StalePublishFromDestroyedSubscriptionStealsNewSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_stale_publish_test");
  const std::string topic = "/battery/stale_publish";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto first_response = registry.renewSubscription("alice", topic, 0, past);
  ASSERT_EQ(published_names.size(), 1U);
  const std::string track_name = first_response.track_name;

  registry.pruneExpiredLeases();
  ASSERT_FALSE(registry.hasSubscription(topic));
  EXPECT_TRUE(unpublished_names.empty());

  const auto second_response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 2U);
  ASSERT_TRUE(registry.hasSubscription(topic));
  EXPECT_EQ(second_response.track_name, track_name);

  const std::size_t new_generation = registry.registryGeneration();
  EXPECT_NE(new_generation, 0U);

  const bool old_publish_accepted = registry.onCdrTrackPublished(track_name, 0);
  EXPECT_FALSE(old_publish_accepted);

  const bool new_publish_accepted = registry.onCdrTrackPublished(track_name, new_generation);
  EXPECT_TRUE(new_publish_accepted);
}

TEST(SubscriptionRegistryTest, StaleDisconnectAfterLeaseExpiryDoesNotTriggerReplay)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_stale_disconnect_test");
  const std::string topic = "/battery/stale_disconnect";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    nullptr);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto first_response = registry.renewSubscription("alice", topic, 0, past);
  ASSERT_TRUE(registry.onCdrTrackPublished(first_response.track_name, 0));

  const std::size_t old_generation = registry.registryGeneration();

  registry.pruneExpiredLeases();
  ASSERT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);

  const std::size_t new_generation = registry.registryGeneration();
  const auto second_response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_TRUE(registry.onCdrTrackPublished(second_response.track_name, new_generation));
  const std::size_t published_count_before = published_names.size();
  const std::size_t unpublished_count_before = unpublished_names.size();

  registry.markRequesterForCdrReplay("alice", old_generation);
  registry.replayCdrTracksForRequester("alice");
  EXPECT_EQ(unpublished_names.size(), unpublished_count_before);
  EXPECT_EQ(published_names.size(), published_count_before);
}

}  // namespace
}  // namespace livekit_ros2_bridge
