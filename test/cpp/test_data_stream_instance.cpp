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
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
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

template <typename PublisherT, typename MessageT>
bool publishUntilFrameCount(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<PublisherT> & publisher,
  const MessageT & message,
  const FakeRoomConnection & session,
  std::size_t expected_count,
  std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  return publishUntil(
    executor,
    publisher,
    message,
    [&]() { return session.state->pushed_data_track_frames.size() == expected_count; },
    timeout);
}

template <typename PublisherT, typename MessageT>
void publishAndDrain(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<PublisherT> & publisher,
  const MessageT & message,
  std::chrono::milliseconds settle_time)
{
  publisher->publish(message);
  executor.spin_some();
  std::this_thread::sleep_for(settle_time);
  executor.spin_some();
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
  const auto message = makeBatteryState();

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 1U));

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(60));
  EXPECT_EQ(session.state->pushed_data_track_frames.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 2U));
}

TEST(DataStreamInstanceTest, RepublishResetsSuppressionBeforeIntervalExpires)
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
  registry.renewSubscription("alice", topic, 1000, kFarFuture);
  const auto message = makeBatteryState();

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 1U));

  registry.markRequesterForDataTrackRepublish("alice", registry.registryGeneration());
  registry.republishDataTracksForRequester("alice");
  // TODO: Keep detailed republish bookkeeping coverage in test_subscription_registry.cpp;
  // this test should stay focused on suppression timing.

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 2U, std::chrono::milliseconds(300)));
}

TEST(DataStreamInstanceTest, RecoversFromPublishFailureWithoutStartingSuppressionWindow)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_publish_failure_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_publish_failure";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

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

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto failed_response = registry.renewSubscription("alice", topic, 500, kFarFuture);
  const auto message = makeBatteryState();

  EXPECT_TRUE(failed_response.track_name.empty());

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(40));
  EXPECT_TRUE(session.state->pushed_data_track_frames.empty());

  const auto recovered_response = registry.renewSubscription("alice", topic, 500, kFarFuture);
  EXPECT_FALSE(recovered_response.track_name.empty());

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 1U, std::chrono::milliseconds(120)));
}

TEST(DataStreamInstanceTest, PendingPublishDoesNotStartSuppressionWindow)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_pending_publish_interval_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/per_instance_pending_publish";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  const auto message = makeBatteryState();
  session.state->publish_data_track_handler =
    [&executor, &publisher, &message](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    // Deliver one ROS message while the track is still pending so the next post-publish
    // message proves whether the suppression window started too early.
    publisher->publish(message);
    executor.spin_some();

    auto owner = std::make_shared<int>(1);
    return std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
  };

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 1000, kFarFuture);

  EXPECT_TRUE(session.state->pushed_data_track_frames.empty());

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 1U, std::chrono::milliseconds(300)));
  EXPECT_EQ(session.state->pushed_data_track_frames[0].track_name, response.track_name);
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
  const auto message = makeBatteryState();

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, session, 1U));

  registry.shutdown();

  EXPECT_FALSE(registry.hasSubscription(topic));
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{response.track_name});

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(60));
  EXPECT_EQ(session.state->pushed_data_track_frames.size(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
