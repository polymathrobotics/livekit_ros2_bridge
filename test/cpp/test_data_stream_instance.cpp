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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "subscription_registry.hpp"

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

TEST(DataStreamInstanceTest, PublishesTrackAndPushesSerializedMessagesThroughOwnedPublisher)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_delivery_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_delivery";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10).best_effort());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);

  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(response.track_name, "ros.data.battery.per_instance_delivery");
  EXPECT_EQ(session.state->published_data_track_names[0], response.track_name);

  const auto message = makeBatteryState();
  ASSERT_TRUE(
    publishUntil(executor, publisher, message, [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  ASSERT_EQ(session.state->pushed_data_track_frames.size(), 1U);
  EXPECT_EQ(session.state->pushed_data_track_frames[0].track_name, response.track_name);
  EXPECT_EQ(
    deserializeMessage<sensor_msgs::msg::BatteryState>(session.state->pushed_data_track_frames[0].payload), message);
}

TEST(DataStreamInstanceTest, SuppressesMessagesAccordingToAppliedInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_interval_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);
  registry.renewSubscription("alice", topic, 150, kFarFuture);

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeBatteryState(), [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(session.state->pushed_data_track_frames.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  ASSERT_TRUE(publishUntil(
    executor, publisher, makeBatteryState(), [&]() { return session.state->pushed_data_track_frames.size() == 2U; }));
}

TEST(DataStreamInstanceTest, RepublishResetsSuppressionAndRequestsNewPublish)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_republish_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_republish";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 1000, kFarFuture);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeBatteryState(), [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  registry.markRequesterForDataTrackRepublish("alice", registry.registryGeneration());
  registry.republishDataTracksForRequester("alice");

  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], response.track_name);
  ASSERT_EQ(session.state->published_data_track_names.size(), 2U);
  EXPECT_EQ(session.state->published_data_track_names[0], session.state->published_data_track_names[1]);

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeBatteryState(), [&]() { return session.state->pushed_data_track_frames.size() == 2U; }));
}

TEST(DataStreamInstanceTest, ShutdownUnpublishesPublishedTrackAndDropsSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_shutdown_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_shutdown";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], response.track_name);

  ASSERT_TRUE(publishUntil(
    executor, publisher, makeBatteryState(), [&]() { return session.state->pushed_data_track_frames.size() == 1U; }));

  registry.shutdown();

  EXPECT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], response.track_name);

  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(session.state->pushed_data_track_frames.size(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
