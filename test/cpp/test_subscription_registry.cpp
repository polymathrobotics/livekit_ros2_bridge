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
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "data_stream_registry.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_registry.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;

const auto kFarFuture = std::chrono::steady_clock::now() + std::chrono::hours(1);

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

TEST(SubscriptionRegistryTest, RenewSubscriptionReturnsDeterministicDataTrackForNonVideoTopics)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);

  const auto first = registry.renewSubscription("alice", "  //battery/state/  ", 0, kFarFuture);
  const auto second = registry.renewSubscription("bob", topic, 0, kFarFuture);

  EXPECT_EQ(first.name, topic);
  EXPECT_EQ(first.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(first.delivery_kind, SubscriptionDeliveryKind::kData);
  EXPECT_EQ(first.track_name, "ros.data.battery.state");
  EXPECT_EQ(second.track_name, first.track_name);
  EXPECT_EQ(session.state->published_data_track_names, std::vector<std::string>{first.track_name});
}

TEST(SubscriptionRegistryTest, PushesRawCdrFramesForDataSubscriptions)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);

  const auto message = makeBatteryState();
  ASSERT_TRUE(
    publishUntil(executor, publisher, message, [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  EXPECT_EQ(session.state->pushed_data_track_frames[0].track_name, response.track_name);
  EXPECT_EQ(
    deserializeMessage<sensor_msgs::msg::BatteryState>(session.state->pushed_data_track_frames[0].payload), message);
}

TEST(SubscriptionRegistryTest, ClampsNegativeRequestedIntervalToZero)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto response = registry.renewSubscription("alice", topic, -25, kFarFuture);

  EXPECT_EQ(response.applied_interval_ms, 0);
}

TEST(SubscriptionRegistryTest, PruneExpiredLeasesKeepsSharedSubscriptionAndRecomputesInterval)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto expired = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  registry.renewSubscription("alice", topic, 50, expired);
  const auto shared = registry.renewSubscription("bob", topic, 300, kFarFuture);
  ASSERT_EQ(shared.applied_interval_ms, 50);

  registry.pruneExpiredLeases();

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_TRUE(session.state->unpublished_data_track_names.empty());
  EXPECT_EQ(registry.renewSubscription("bob", topic, 300, kFarFuture).applied_interval_ms, 300);
}

TEST(SubscriptionRegistryTest, OmittedRequesterTargetExpiresWhileRenewedSiblingTargetStaysActive)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);

  const auto initial_expiry = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  registry.renewSubscription("alice", topic_a, 0, initial_expiry);
  registry.renewSubscription("alice", topic_b, 0, initial_expiry);

  registry.renewSubscription("alice", topic_a, 0, kFarFuture);

  ASSERT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_a) != nullptr);
  ASSERT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_b) != nullptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  registry.pruneExpiredLeases();

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_a) != nullptr);
  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic_b) != nullptr);
}

TEST(SubscriptionRegistryTest, CreatesVideoSubscriptionsForRosTopicsAndConfiguredSources)
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
  SubscriptionRegistry registry(*node, data_stream_registry, &video_stream_registry, &video_stream_config);

  const auto topic_response = registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  const auto source_response = registry.renewSubscription(
    "bob", SubscriptionDemand{SubscriptionTargetKind::ConfiguredSource, "/sources/front", std::nullopt}, kFarFuture);

  EXPECT_EQ(topic_response.delivery_kind, SubscriptionDeliveryKind::kVideo);
  EXPECT_EQ(topic_response.track_name, "ros.video.camera.front");
  EXPECT_EQ(source_response.delivery_kind, SubscriptionDeliveryKind::kVideo);
  EXPECT_FALSE(source_response.track_name.empty());
}

TEST(SubscriptionRegistryTest, ParticipantRefreshRepublishesPublishedDataTrackWithoutDroppingLease)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 1000, kFarFuture);

  registry.queueDataTrackRepublish("alice", data_stream_registry.generation());
  registry.republishDataTracks("alice");

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{response.track_name});
  EXPECT_EQ(
    session.state->published_data_track_names, (std::vector<std::string>{response.track_name, response.track_name}));
}

TEST(SubscriptionRegistryTest, NewRequesterRepublishesAlreadyPublishedDataTrack)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto first = registry.renewSubscription("alice", topic, 1000, kFarFuture);
  const auto second = registry.renewSubscription("bob", topic, 250, kFarFuture);
  EXPECT_EQ(second.track_name, first.track_name);

  registry.republishDataTracks("bob");

  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{first.track_name});
  EXPECT_EQ(session.state->published_data_track_names, (std::vector<std::string>{first.track_name, first.track_name}));
}

TEST(SubscriptionRegistryTest, StaleGenerationDoesNotRepublishReplacementSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_stale_republish_generation_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/stale_republish_generation";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto first = registry.renewSubscription("alice", topic, 0, kFarFuture);
  const auto stale_generation = data_stream_registry.generation();

  registry.revokeRequesterLeases("alice");
  ASSERT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);

  const auto replacement = registry.renewSubscription("alice", topic, 0, kFarFuture);

  registry.queueDataTrackRepublish("alice", stale_generation);
  registry.republishDataTracks("alice");

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(
    session.state->published_data_track_names, (std::vector<std::string>{first.track_name, replacement.track_name}));
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{first.track_name});
}

TEST(SubscriptionRegistryTest, RevokeRequesterLeasesPreservesSharedSubscriptionsOwnedByOthers)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_revoke_shared_test");
  FakeRoomConnection session;
  VideoStreamRegistry video_stream_registry(*node, session);
  const std::string alice_only_topic = "/battery/alice_only";
  const std::string shared_data_topic = "/battery/shared";
  const std::string shared_video_topic = "/camera/shared";
  auto alice_only_pub = node->create_publisher<sensor_msgs::msg::BatteryState>(alice_only_topic, rclcpp::QoS(10));
  auto shared_data_pub = node->create_publisher<sensor_msgs::msg::BatteryState>(shared_data_topic, rclcpp::QoS(10));
  auto shared_video_pub = node->create_publisher<sensor_msgs::msg::Image>(shared_video_topic, rclcpp::QoS(10));
  (void)alice_only_pub;
  (void)shared_data_pub;
  (void)shared_video_pub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, alice_only_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, shared_data_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, shared_video_topic, "sensor_msgs/msg/Image"));

  DataStreamRegistry data_stream_registry(*node, session);
  SubscriptionRegistry registry(*node, data_stream_registry, &video_stream_registry);
  const auto alice_only = registry.renewSubscription("alice", alice_only_topic, 50, kFarFuture);
  registry.renewSubscription("alice", shared_data_topic, 50, kFarFuture);
  registry.renewSubscription("bob", shared_data_topic, 250, kFarFuture);
  registry.renewSubscription("alice", shared_video_topic, 0, kFarFuture);
  registry.renewSubscription("bob", shared_video_topic, 0, kFarFuture);

  registry.revokeRequesterLeases("alice");

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, alice_only_topic) != nullptr);
  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, shared_data_topic) != nullptr);
  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, shared_video_topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{alice_only.track_name});
}

TEST(SubscriptionRegistryTest, RevokeRequesterLeasesClearsQueuedRepublishForRequester)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_revoke_clears_republish_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/revoke_clears_republish";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry data_stream_registry(*node, session);
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 1000, kFarFuture);
  registry.renewSubscription("bob", topic, 1000, kFarFuture);

  registry.queueDataTrackRepublish("alice", data_stream_registry.generation());
  registry.revokeRequesterLeases("alice");
  registry.republishDataTracks("alice");

  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_TRUE(session.state->unpublished_data_track_names.empty());
  EXPECT_EQ(session.state->published_data_track_names, std::vector<std::string>{response.track_name});
}

TEST(SubscriptionRegistryTest, PruneExpiredLeasesUnpublishesPublishedTrack)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto response = registry.renewSubscription("alice", topic, 0, past);

  registry.pruneExpiredLeases();

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{response.track_name});
}

TEST(SubscriptionRegistryTest, ResetSessionStateClearsDataAndVideoSubscriptions)
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
  SubscriptionRegistry registry(*node, data_stream_registry, &video_stream_registry);
  const auto response = registry.renewSubscription("alice", data_topic, 0, kFarFuture);
  registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  ASSERT_TRUE(publishUntil(
    executor, video_pub, makeRgbImage(), [&]() { return session.state->published_video_track_names.size() == 1U; }));

  registry.resetSessionState();

  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, data_topic) != nullptr);
  EXPECT_FALSE(registry.findSubscription(SubscriptionTargetKind::Topic, video_topic) != nullptr);
  EXPECT_FALSE(data_stream_registry.onTrackPublished(response.track_name, data_stream_registry.generation()));
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{response.track_name});
  EXPECT_EQ(session.state->unpublished_video_track_names, std::vector<std::string>{"ros.video.camera.reset"});
}

TEST(SubscriptionRegistryTest, ShutdownWaitsForActiveSerializedMessageCallback)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  registry.renewSubscription("alice", topic, 0, kFarFuture);

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
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{"ros.data.battery.shutdown_quiesce"});

  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionRegistryTest, QueueFullPushLeavesSubscriptionActive)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  registry.renewSubscription("alice", topic, 0, kFarFuture);

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return push_attempt_count.load() >= 1; }));
  EXPECT_TRUE(registry.findSubscription(SubscriptionTargetKind::Topic, topic) != nullptr);
}

TEST(SubscriptionRegistryTest, RequesterSpecificMethodsRejectEmptyIdentity)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_empty_requester_test");
  FakeRoomConnection session;
  DataStreamRegistry data_stream_registry(*node, session);
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);

  expectInvalidArgumentMessage(
    [&registry]() { (void)registry.renewSubscription("", "/some/topic", 0, kFarFuture); },
    "requester_identity is required");
  expectInvalidArgumentMessage(
    [&registry]() { registry.queueDataTrackRepublish("", 0); }, "requester_identity is required");
  expectInvalidArgumentMessage([&registry]() { registry.republishDataTracks(""); }, "requester_identity is required");
  expectInvalidArgumentMessage([&registry]() { registry.revokeRequesterLeases(""); }, "requester_identity is required");
}

TEST(SubscriptionRegistryTest, ShutdownRejectsNewSubscriptionsAndFurtherLifecycleCallsAreNoOps)
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
  SubscriptionRegistry registry(*node, data_stream_registry, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);

  registry.shutdown();

  try {
    (void)registry.renewSubscription("alice", topic, 0, kFarFuture);
    FAIL() << "Expected StreamUnavailableError";
  } catch (const StreamUnavailableError & exc) {
    EXPECT_STREQ(exc.what(), "Subscription registry is shut down.");
  } catch (const std::exception & exc) {
    FAIL() << "Expected StreamUnavailableError, got: " << exc.what();
  } catch (...) {
    FAIL() << "Expected StreamUnavailableError";
  }

  registry.pruneExpiredLeases();
  registry.resetSessionState();

  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{response.track_name});
}

}  // namespace
}  // namespace livekit_ros2_bridge
