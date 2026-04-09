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
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "subscription_registry.hpp"
#include "video_sidecar_supervisor.hpp"

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

VideoSidecarSupervisor::Config makeTestConfig()
{
  VideoSidecarSupervisor::Config config;
  config.livekit_url = "ws://localhost:7880";
  config.livekit_room = "test-room";
  config.api_key = "test-api-key";
  config.api_secret = "test-api-secret";
  config.token_ttl = std::chrono::seconds(600);
  config.bridge_identity = "bridge-test";
  return config;
}

VideoConfig makeConfiguredVideoConfig()
{
  VideoConfig config = makeDefaultVideoConfig();

  config.pipeline_sources.emplace(
    "/sources/front",
    ConfiguredPipelineSource{
      "uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0 ! videoconvert ! vp8enc deadline=1"});
  return config;
}

std::vector<std::string> fakeSidecarCommandBuilder(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token)
{
  (void)spec;
  (void)livekit_url;
  (void)livekit_token;
  return {"sleep", "3600"};
}

std::string makeRosSidecarKey(const std::string & topic, const std::string & interface_type)
{
  return resolveRosVideoLaunchSpec(makeDefaultVideoConfig(), topic, interface_type).sidecar_key;
}

std::string makeConfiguredSidecarKey(const VideoConfig & config, const std::string & external_name)
{
  return resolvePipelineVideoLaunchSpec(config, external_name).sidecar_key;
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
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kDataTrack);
  ASSERT_EQ(published_track_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_track_names[0]);
  EXPECT_EQ(second_response.target.name, topic);
  EXPECT_EQ(second_response.interface_type, "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(second_response.delivery_kind, StreamDeliveryKind::kDataTrack);
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
  EXPECT_EQ(raw_response.delivery_kind, StreamDeliveryKind::kDataTrack);
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
  EXPECT_EQ(cdr_frames[0].track_name, "ros.cdr.battery.send");
  const auto decoded = deserializeMessage<sensor_msgs::msg::BatteryState>(cdr_frames[0].data);
  EXPECT_EQ(decoded, message);
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
  const std::string topic = "/camera/front";
  const std::string requested_topic = "  camera/front  ";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  const auto response = registry.renewSubscription("alice", requested_topic, 0, kFarFuture);
  const auto second_response = registry.renewSubscription("bob", topic, 0, kFarFuture);

  EXPECT_EQ(response.target.name, topic);
  EXPECT_EQ(response.interface_type, "sensor_msgs/msg/Image");
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_EQ(response.publisher_identity, "bridge-test-video-camera-front");
  EXPECT_TRUE(response.track_name.empty());
  EXPECT_EQ(response.applied_interval_ms, 0);
  EXPECT_EQ(second_response.target.name, topic);
  EXPECT_EQ(second_response.interface_type, "sensor_msgs/msg/Image");
  EXPECT_EQ(second_response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_EQ(second_response.publisher_identity, "bridge-test-video-camera-front");
}

TEST(SubscriptionRegistryTest, RenewSubscriptionCreatesConfiguredSourceSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_configured_source_test");

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const VideoConfig video_config = makeConfiguredVideoConfig();
  const std::string sidecar_key = makeConfiguredSidecarKey(video_config, "/sources/front");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor, &video_config);

  const auto response = registry.renewSubscription(
    "alice", SubscriptionRequest{{SubscriptionTargetKind::External, "/sources/front"}, std::nullopt}, kFarFuture);

  EXPECT_EQ(response.target.kind, SubscriptionTargetKind::External);
  EXPECT_EQ(response.target.name, "/sources/front");
  EXPECT_EQ(response.interface_type, "");
  EXPECT_EQ(response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_EQ(response.publisher_identity, "bridge-test-video-source-sources-front");
  EXPECT_TRUE(response.track_name.empty());
  EXPECT_TRUE(registry.hasSubscription("/sources/front", SubscriptionTargetKind::External));
  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));
}

TEST(SubscriptionRegistryTest, RenewSubscriptionNormalizesRawConfiguredSourceHeartbeatSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_raw_configured_source_test");

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const VideoConfig video_config = makeConfiguredVideoConfig();
  const std::string external_name = "/sources/front";
  const std::string sidecar_key = makeConfiguredSidecarKey(video_config, external_name);
  const SubscriptionRequest raw_subscription{{SubscriptionTargetKind::External, "  //sources//front/  "}, std::nullopt};
  const SubscriptionRequest canonical_subscription{{SubscriptionTargetKind::External, external_name}, std::nullopt};

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor, &video_config);

  const auto raw_response = registry.renewSubscription("alice", raw_subscription, kFarFuture);
  const auto canonical_response = registry.renewSubscription("bob", canonical_subscription, kFarFuture);

  EXPECT_EQ(raw_response.target.name, external_name);
  EXPECT_EQ(canonical_response.target.name, external_name);
  EXPECT_EQ(raw_response.publisher_identity, canonical_response.publisher_identity);
  EXPECT_TRUE(registry.hasSubscription(external_name, SubscriptionTargetKind::External));
  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  registry.removeRequesterLeases("bob");

  EXPECT_TRUE(registry.hasSubscription(external_name, SubscriptionTargetKind::External));
  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  registry.removeRequesterLeases("alice");

  EXPECT_FALSE(registry.hasSubscription(external_name, SubscriptionTargetKind::External));
  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
}

TEST(SubscriptionRegistryTest, TopicAndConfiguredSourceStayDistinctWhenNamesMatch)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_distinct_target_names_test");
  const std::string shared_name = "/sources/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(shared_name, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, shared_name, "sensor_msgs/msg/BatteryState"));

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const VideoConfig video_config = makeConfiguredVideoConfig();
  const std::string sidecar_key = makeConfiguredSidecarKey(video_config, shared_name);

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor, &video_config);

  const auto topic_response = registry.renewSubscription("alice", shared_name, 0, kFarFuture);
  const auto source_response = registry.renewSubscription(
    "bob", SubscriptionRequest{{SubscriptionTargetKind::External, shared_name}, std::nullopt}, kFarFuture);

  EXPECT_EQ(topic_response.target.kind, SubscriptionTargetKind::Topic);
  EXPECT_EQ(topic_response.target.name, shared_name);
  EXPECT_EQ(topic_response.delivery_kind, StreamDeliveryKind::kDataTrack);
  EXPECT_EQ(source_response.target.kind, SubscriptionTargetKind::External);
  EXPECT_EQ(source_response.target.name, shared_name);
  EXPECT_EQ(source_response.delivery_kind, StreamDeliveryKind::kVideo);
  EXPECT_TRUE(registry.hasSubscription(shared_name, SubscriptionTargetKind::Topic));
  EXPECT_TRUE(registry.hasSubscription(shared_name, SubscriptionTargetKind::External));
  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));
}

TEST(SubscriptionRegistryTest, FailedVideoRestartRenewalDoesNotExtendTopicLifetime)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_video_restart_lease_rollback_test");
  const std::string topic = "/camera/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  bool fail_restart = false;
  auto flaky_sidecar_command_builder =
    [&fail_restart](const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
    if (fail_restart) {
      return {};
    }
    return {"sleep", "3600"};
  };

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), flaky_sidecar_command_builder);
  const std::string sidecar_key = makeRosSidecarKey(topic, "sensor_msgs/msg/Image");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  const auto short_lease = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  registry.renewSubscription("alice", topic, 0, short_lease);
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));
  ASSERT_TRUE(registry.hasSubscription(topic));

  supervisor.stopSidecar(sidecar_key);
  ASSERT_FALSE(supervisor.isSidecarRunning(sidecar_key));

  fail_restart = true;
  EXPECT_THROW(registry.renewSubscription("bob", topic, 0, kFarFuture), StreamUnavailableError);
  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
  EXPECT_TRUE(registry.hasSubscription(topic));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  registry.sweepExpiredLeases();

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
}

TEST(SubscriptionRegistryTest, ThrowsUnavailableWhenNoVideoSidecarSupervisor)
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

TEST(SubscriptionRegistryTest, SweepExpiredLeasesStopsVideoProcess)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_sweep_video_test");
  const std::string topic = "/camera/front";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const std::string sidecar_key = makeRosSidecarKey(topic, "sensor_msgs/msg/Image");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  registry.renewSubscription("alice", topic, 0, past);
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  registry.sweepExpiredLeases();

  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
  EXPECT_FALSE(registry.hasSubscription(topic));
}

TEST(SubscriptionRegistryTest, SweepExpiredLeasesRestartsUnhealthyVideoSidecar)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_restart_unhealthy_video_test");
  const std::string topic = "/camera/restart_unhealthy";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  int spawn_count = 0;
  bool healthy = true;
  auto counting_builder =
    [&spawn_count](const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
    ++spawn_count;
    return {"sleep", "3600"};
  };

  auto config = makeTestConfig();
  config.health_check_startup_grace = std::chrono::milliseconds::zero();
  config.unhealthy_restart_threshold = 1U;
  VideoSidecarSupervisor supervisor(
    std::move(config), counting_builder, [&healthy](const std::string &) { return healthy; });
  const std::string sidecar_key = makeRosSidecarKey(topic, "sensor_msgs/msg/Image");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));
  ASSERT_EQ(spawn_count, 1);

  healthy = false;
  registry.sweepExpiredLeases();

  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));
  EXPECT_TRUE(registry.hasSubscription(topic));
  EXPECT_EQ(spawn_count, 2);
}

TEST(SubscriptionRegistryTest, FailedUnhealthyVideoRestartKeepsSubscriptionAndSidecarAlive)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_unhealthy_restart_failure_test");
  const std::string topic = "/camera/restart_unhealthy_failure";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  bool healthy = true;
  bool fail_restart = false;
  auto flaky_builder =
    [&fail_restart](const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
    if (fail_restart) {
      return {};
    }
    return {"sleep", "3600"};
  };

  auto config = makeTestConfig();
  config.health_check_startup_grace = std::chrono::milliseconds::zero();
  config.unhealthy_restart_threshold = 1U;
  VideoSidecarSupervisor supervisor(
    std::move(config), flaky_builder, [&healthy](const std::string &) { return healthy; });
  const std::string sidecar_key = makeRosSidecarKey(topic, "sensor_msgs/msg/Image");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  const auto short_lease = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  registry.renewSubscription("alice", topic, 0, short_lease);
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));
  ASSERT_TRUE(registry.hasSubscription(topic));

  healthy = false;
  fail_restart = true;
  registry.sweepExpiredLeases();

  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));
  EXPECT_TRUE(registry.hasSubscription(topic));

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  registry.sweepExpiredLeases();

  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
  EXPECT_FALSE(registry.hasSubscription(topic));
}

TEST(SubscriptionRegistryTest, RemoveRequesterLeasesPreservesSharedSubscriptionsOwnedByOthers)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_remove_requester_shared_test");
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

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const std::string shared_video_sidecar_key = makeRosSidecarKey(shared_video_topic, "sensor_msgs/msg/Image");

  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    noopCdrPublish(),
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    &supervisor);

  const auto alice_only = registry.renewSubscription("alice", alice_only_topic, 50, kFarFuture);
  const auto shared_data = registry.renewSubscription("alice", shared_data_topic, 50, kFarFuture);
  registry.renewSubscription("bob", shared_data_topic, 250, kFarFuture);
  const auto shared_video = registry.renewSubscription("alice", shared_video_topic, 0, kFarFuture);
  registry.renewSubscription("bob", shared_video_topic, 0, kFarFuture);
  ASSERT_TRUE(registry.onCdrTrackPublished(alice_only.track_name, 0));
  ASSERT_TRUE(registry.onCdrTrackPublished(shared_data.track_name, 0));
  ASSERT_TRUE(supervisor.isSidecarRunning(shared_video_sidecar_key));

  registry.removeRequesterLeases("alice");

  EXPECT_FALSE(registry.hasSubscription(alice_only_topic));
  EXPECT_TRUE(registry.hasSubscription(shared_data_topic));
  EXPECT_TRUE(registry.hasSubscription(shared_video_topic));
  EXPECT_TRUE(supervisor.isSidecarRunning(shared_video_sidecar_key));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], alice_only.track_name);

  const auto shared_data_after_disconnect = registry.renewSubscription("bob", shared_data_topic, 250, kFarFuture);
  EXPECT_EQ(shared_data_after_disconnect.track_name, shared_data.track_name);
  EXPECT_EQ(shared_data_after_disconnect.applied_interval_ms, 250);

  const auto shared_video_after_disconnect = registry.renewSubscription("bob", shared_video_topic, 0, kFarFuture);
  EXPECT_EQ(shared_video_after_disconnect.publisher_identity, shared_video.publisher_identity);
}

TEST(SubscriptionRegistryTest, RemoveRequesterLeasesStopsRequesterOwnedVideoSidecarImmediately)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_remove_requester_video_test");
  const std::string topic = "/camera/remove_requester";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/Image"));

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const std::string sidecar_key = makeRosSidecarKey(topic, "sensor_msgs/msg/Image");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), &supervisor);

  registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_TRUE(registry.hasSubscription(topic));
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  registry.removeRequesterLeases("alice");

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
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

  registry.replayCdrTracksForRequester("alice");
  EXPECT_EQ(published_names.size(), 2U);
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

  registry.replayCdrTracksForRequester("bob");
  EXPECT_EQ(published_names.size(), 2U);
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

  registry.removeRequesterLeases("alice");
  const auto disconnect_after_prune = registry.renewSubscription("bob", disconnect_topic, 250, kFarFuture);
  EXPECT_EQ(disconnect_after_prune.track_name, disconnect_initial.track_name);
  EXPECT_EQ(disconnect_after_prune.applied_interval_ms, 250);

  registry.sweepExpiredLeases();
  const auto expiry_after_prune = registry.renewSubscription("dave", expiry_topic, 250, kFarFuture);
  EXPECT_EQ(expiry_after_prune.track_name, expiry_initial.track_name);
  EXPECT_EQ(expiry_after_prune.applied_interval_ms, 250);
}

TEST(SubscriptionRegistryTest, SweepExpiredLeasesKeepsRunningVideoProcessOnRefreshPreparationFailure)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_refresh_failure_test");
  const std::string data_topic = "/battery/refresh_failure";
  const std::string video_topic = "/camera/refresh_failure";
  auto data_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(data_topic, rclcpp::QoS(10));
  auto video_publisher = node->create_publisher<sensor_msgs::msg::Image>(video_topic, rclcpp::QoS(10));
  (void)data_publisher;
  (void)video_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, data_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, video_topic, "sensor_msgs/msg/Image"));

  bool fail_video_refresh = false;
  auto flaky_sidecar_command_builder =
    [&fail_video_refresh](
      const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
    if (fail_video_refresh) {
      throw std::runtime_error("refresh failed");
    }
    return {"sleep", "3600"};
  };

  auto config = makeTestConfig();
  config.token_ttl = std::chrono::seconds(2);
  config.token_refresh_margin = std::chrono::seconds(1);
  VideoSidecarSupervisor supervisor(std::move(config), flaky_sidecar_command_builder);
  const std::string sidecar_key = makeRosSidecarKey(video_topic, "sensor_msgs/msg/Image");

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    &supervisor);

  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  registry.renewSubscription("alice", data_topic, 0, past);
  ASSERT_EQ(published_names.size(), 1U);
  registry.onCdrTrackPublished(published_names[0], 0);

  registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  ASSERT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  fail_video_refresh = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  EXPECT_NO_THROW(registry.sweepExpiredLeases());

  EXPECT_FALSE(registry.hasSubscription(data_topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], published_names[0]);
  EXPECT_TRUE(registry.hasSubscription(video_topic));
  EXPECT_TRUE(supervisor.isSidecarRunning(sidecar_key));

  registry.shutdown();
}

TEST(SubscriptionRegistryTest, ResetSessionStateClearsDataAndVideoSubscriptions)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_reset_test");
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

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const std::string video_sidecar_key = makeRosSidecarKey(video_topic, "sensor_msgs/msg/Image");

  std::vector<std::string> published_track_names;
  std::vector<std::string> unpublished_track_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_track_names](const std::string & name, std::size_t) { published_track_names.push_back(name); },
    [&unpublished_track_names](const std::string & name) { unpublished_track_names.push_back(name); },
    &supervisor);

  const auto response = registry.renewSubscription("alice", data_topic, 0, kFarFuture);
  registry.renewSubscription("alice", video_topic, 0, kFarFuture);
  ASSERT_EQ(published_track_names.size(), 1U);
  EXPECT_EQ(response.track_name, published_track_names[0]);
  EXPECT_TRUE(registry.onCdrTrackPublished(published_track_names[0], 0));
  ASSERT_TRUE(registry.hasSubscription(data_topic));
  ASSERT_TRUE(registry.hasSubscription(video_topic));
  ASSERT_TRUE(supervisor.isSidecarRunning(video_sidecar_key));

  registry.resetSessionState();

  EXPECT_FALSE(registry.hasSubscription(data_topic));
  EXPECT_FALSE(registry.hasSubscription(video_topic));
  EXPECT_FALSE(supervisor.isSidecarRunning(video_sidecar_key));
  ASSERT_EQ(unpublished_track_names.size(), 1U);
  EXPECT_EQ(unpublished_track_names[0], published_track_names[0]);
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

  registry.sweepExpiredLeases();

  EXPECT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], published_names[0]);
}

TEST(SubscriptionRegistryTest, SweepExpiredLeasesDoesNotUnpublishPendingCdrTrack)
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

  registry.sweepExpiredLeases();

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_TRUE(unpublished_names.empty());
}

TEST(SubscriptionRegistryTest, OnCdrTrackPublishedReturnsFalseForUnknownTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_cdr_unknown_track_test");

  SubscriptionRegistry registry(*node, noopCdrSend(), noopCdrPublish(), noopCdrUnpublish(), nullptr);

  EXPECT_FALSE(registry.onCdrTrackPublished("ros.cdr.no.such.topic", 0));
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

  std::vector<std::string> published_track_names;
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const VideoConfig video_config = makeConfiguredVideoConfig();
  const std::string sidecar_key = makeConfiguredSidecarKey(video_config, "/sources/front");

  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_track_names](const std::string & name, std::size_t) { published_track_names.push_back(name); },
    noopCdrUnpublish(),
    &supervisor,
    &video_config);

  const auto expect_invalid_argument = [&](const SubscriptionRequest & subscription, const char * expected_message) {
    expectInvalidArgumentMessage(
      [&registry, &subscription]() { (void)registry.renewSubscription("alice", subscription, kFarFuture); },
      expected_message);
  };

  expect_invalid_argument(
    SubscriptionRequest{{SubscriptionTargetKind::Topic, "   "}, std::nullopt},
    "heartbeat subscription target name must normalize to a non-empty topic name");
  expect_invalid_argument(
    SubscriptionRequest{{SubscriptionTargetKind::External, "  \t\n  "}, std::nullopt},
    "heartbeat subscription target name must normalize to a non-empty external name");

  EXPECT_TRUE(published_track_names.empty());
  EXPECT_FALSE(registry.hasSubscription("/sources/front", SubscriptionTargetKind::External));
  EXPECT_FALSE(supervisor.isSidecarRunning(sidecar_key));
}

TEST(SubscriptionRegistryTest, ShutdownClearsVideoSubscriptionsAndUnpublishesPublishedDataTracks)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("subscription_registry_shutdown_test");
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

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), fakeSidecarCommandBuilder);
  const std::string video_sidecar_key = makeRosSidecarKey(video_topic, "sensor_msgs/msg/Image");

  std::vector<std::string> published_names;
  std::vector<std::string> unpublished_names;
  SubscriptionRegistry registry(
    *node,
    noopCdrSend(),
    [&published_names](const std::string & name, std::size_t) { published_names.push_back(name); },
    [&unpublished_names](const std::string & name) { unpublished_names.push_back(name); },
    &supervisor);

  registry.renewSubscription("alice", published_topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 1U);
  registry.onCdrTrackPublished(published_names[0], 0);

  registry.renewSubscription("alice", pending_topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 2U);
  registry.renewSubscription("alice", video_topic, 0, kFarFuture);

  ASSERT_TRUE(registry.hasSubscription(published_topic));
  ASSERT_TRUE(registry.hasSubscription(pending_topic));
  ASSERT_TRUE(registry.hasSubscription(video_topic));
  ASSERT_TRUE(supervisor.isSidecarRunning(video_sidecar_key));

  registry.shutdown();

  EXPECT_FALSE(registry.hasSubscription(published_topic));
  EXPECT_FALSE(registry.hasSubscription(pending_topic));
  EXPECT_FALSE(registry.hasSubscription(video_topic));
  EXPECT_FALSE(supervisor.isSidecarRunning(video_sidecar_key));
  ASSERT_EQ(unpublished_names.size(), 1U);
  EXPECT_EQ(unpublished_names[0], published_names[0]);
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
  EXPECT_FALSE(registry.hasSubscription(topic));
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
  EXPECT_EQ(send_call_count.load(), 2);

  registry.shutdown();
  executor.cancel();
  spin_thread.join();
}

TEST(SubscriptionRegistryTest, StalePublishFromDestroyedSubscriptionStealsNewSubscription)
{
  // Proves the bug: when a subscription is destroyed while its CDR track is kPending,
  // the queued publishTrack work item survives. If a new subscription for the same topic
  // is created before the old publish completes, onCdrTrackPublished finds the NEW
  // subscription (same track name via deriveTrackName) and transitions it to kPublished
  // with the stale handle. The new subscription's publish then sees kPublished (not
  // kPending), gets rejected as an orphan, and data stops flowing.
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

  // Step 1: Create subscription. CDR track is kPending (we do NOT call onCdrTrackPublished).
  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto first_response = registry.renewSubscription("alice", topic, 0, past);
  ASSERT_EQ(published_names.size(), 1U);
  const std::string track_name = first_response.track_name;

  // Step 2: Sweep expired leases. Subscription destroyed while kPending.
  // destroyResource does NOT call unpublish for kPending tracks.
  registry.sweepExpiredLeases();
  ASSERT_FALSE(registry.hasSubscription(topic));
  EXPECT_TRUE(unpublished_names.empty());

  // Step 3: User comes back. Create a NEW subscription for the same topic.
  const auto second_response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(published_names.size(), 2U);
  ASSERT_TRUE(registry.hasSubscription(topic));
  // Both subscriptions produced the same track name (deriveTrackName is deterministic).
  EXPECT_EQ(second_response.track_name, track_name);

  // The old publish was initiated at generation 0. The destroy incremented generation.
  const std::size_t new_generation = registry.registryGeneration();
  EXPECT_NE(new_generation, 0U) << "Generation should have advanced after destroy";

  // Step 4: The OLD queued publish completes with old generation 0.
  const bool old_publish_accepted = registry.onCdrTrackPublished(track_name, 0);
  EXPECT_FALSE(old_publish_accepted) << "Stale publish from destroyed subscription should be rejected";

  // Step 5: The NEW publish completes with the current generation.
  const bool new_publish_accepted = registry.onCdrTrackPublished(track_name, new_generation);
  EXPECT_TRUE(new_publish_accepted) << "Publish for the current subscription should be accepted";
}

TEST(SubscriptionRegistryTest, StaleDisconnectAfterLeaseExpiryDoesNotTriggerReplay)
{
  // Proves the bug: when a user reconnects after lease expiry, a delayed
  // onParticipantDisconnected for the OLD session (same identity) marks the user's
  // fresh subscriptions for CDR replay. The next replayCdrTracksForRequester call
  // unnecessarily unpublishes and republishes CDR tracks.
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

  // Step 1: Create subscription and publish CDR track.
  const auto past = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto first_response = registry.renewSubscription("alice", topic, 0, past);
  ASSERT_TRUE(registry.onCdrTrackPublished(first_response.track_name, 0));

  // Snapshot the generation BEFORE lease expiry (simulates the disconnect callback
  // capturing the generation on the SDK thread before the work queue processes the sweep).
  const std::size_t old_generation = registry.registryGeneration();

  // Step 2: Lease expires. Subscription destroyed, CDR track unpublished.
  registry.sweepExpiredLeases();
  ASSERT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(unpublished_names.size(), 1U);

  // Step 3: User reconnects. New subscription created, CDR track published.
  const std::size_t new_generation = registry.registryGeneration();
  const auto second_response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_TRUE(registry.onCdrTrackPublished(second_response.track_name, new_generation));
  const std::size_t published_count_before = published_names.size();
  const std::size_t unpublished_count_before = unpublished_names.size();

  // Step 4: Delayed onParticipantDisconnected for the OLD session fires with old generation.
  registry.markRequesterForCdrReplay("alice", old_generation);

  // Step 5: replayCdrTracksForRequester should NOT replay because the disconnect was
  // for the old session. The fresh subscription should be left alone.
  registry.replayCdrTracksForRequester("alice");
  EXPECT_EQ(unpublished_names.size(), unpublished_count_before)
    << "Fresh CDR track should not be unpublished due to stale disconnect";
  EXPECT_EQ(published_names.size(), published_count_before)
    << "Fresh CDR track should not be republished due to stale disconnect";
}

}  // namespace
}  // namespace livekit_ros2_bridge
