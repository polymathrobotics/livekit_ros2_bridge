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
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "data_track_publisher.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;

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

const char * batteryStateInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::BatteryState>();
}

std::shared_ptr<DataTrackPublisher> createDataTrackPublisher(
  rclcpp::Node & node, FakeRoomConnection & room_connection, const std::string & ros_topic)
{
  return std::make_shared<DataTrackPublisher>(
    ros_topic,
    batteryStateInterfaceType(),
    node.get_node_topics_interface(),
    node.get_node_graph_interface(),
    node.get_clock(),
    room_connection,
    nullptr);
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
  const FakeRoomConnection & room_connection,
  std::size_t expected_count,
  std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  return publishUntil(
    executor,
    publisher,
    message,
    [&]() { return room_connection.state->pushed_data_track_frames.size() == expected_count; },
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

TEST(DataTrackPublisherTest, SuppressesMessagesAccordingToAppliedInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_interval_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_interval";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->setIntervalMs(150);
  track_publisher->publish();
  const auto message = makeBatteryState();

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, room_connection, 1U));

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(60));
  EXPECT_EQ(room_connection.state->pushed_data_track_frames.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, room_connection, 2U));
}

TEST(DataTrackPublisherTest, RecoversFromPublishFailureWithoutStartingSuppressionWindow)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_publish_failure_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_publish_failure";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  int publish_attempt_count = 0;
  room_connection.state->publish_data_track_handler =
    [&publish_attempt_count, &room_connection](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    publish_attempt_count++;
    if (publish_attempt_count == 1) {
      throw std::runtime_error("simulated publish failure");
    }

    return room_connection.makeSyntheticDataTrack();
  };

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->setIntervalMs(500);
  track_publisher->publish();
  const auto message = makeBatteryState();

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(40));
  EXPECT_TRUE(room_connection.state->pushed_data_track_frames.empty());

  track_publisher->publish();

  ASSERT_TRUE(
    publishUntilFrameCount(executor, publisher, message, room_connection, 1U, std::chrono::milliseconds(120)));
}

TEST(DataTrackPublisherTest, ReentrantPublishDoesNotStartSuppressionWindow)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_pending_publish_interval_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_pending_publish";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  const auto message = makeBatteryState();
  room_connection.state->publish_data_track_handler =
    [&executor, &publisher, &message, &room_connection](
      const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    publisher->publish(message);
    executor.spin_some();

    return room_connection.makeSyntheticDataTrack();
  };

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->setIntervalMs(1000);
  track_publisher->publish();

  EXPECT_TRUE(room_connection.state->pushed_data_track_frames.empty());

  ASSERT_TRUE(
    publishUntilFrameCount(executor, publisher, message, room_connection, 1U, std::chrono::milliseconds(300)));
}

TEST(DataTrackPublisherTest, DestructionAllowsSameTopicToBeRepublishedWithSameTrackName)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_recreate_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_recreate";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  auto first = createDataTrackPublisher(*node, room_connection, topic);
  first->publish();
  const std::string track_name = first->trackName();

  first.reset();

  auto second = createDataTrackPublisher(*node, room_connection, topic);
  second->publish();
  EXPECT_EQ(second->trackName(), track_name);
  EXPECT_EQ(room_connection.state->published_data_track_names, (std::vector<std::string>{track_name, track_name}));
}

TEST(DataTrackPublisherTest, DestructionWaitsForActiveSerializedMessageCallback)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_shutdown_quiesce_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_shutdown_quiesce";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  auto push_entered = std::make_shared<std::promise<void>>();
  auto push_entered_future = push_entered->get_future();
  auto release_push = std::make_shared<std::promise<void>>();
  auto release_push_future = release_push->get_future().share();
  room_connection.state->try_push_data_track_handler = [push_entered, release_push_future](
                                                         const std::string &, const livekit::DataTrackFrame &) {
    push_entered->set_value();
    release_push_future.wait();
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
  };

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->publish();
  const std::string track_name = track_publisher->trackName();

  std::thread spin_thread([&executor]() { executor.spin(); });

  publisher->publish(makeBatteryState());
  EXPECT_EQ(push_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto destroy_future = std::async(
    std::launch::async, [track_publisher = std::move(track_publisher)]() mutable { track_publisher.reset(); });
  EXPECT_EQ(destroy_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  release_push->set_value();

  EXPECT_EQ(destroy_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  destroy_future.get();
  EXPECT_EQ(room_connection.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  executor.cancel();
  spin_thread.join();
}

TEST(DataTrackPublisherTest, DestructionUnpublishesPublishedTrackAndDropsSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_shutdown_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_shutdown";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->publish();
  const std::string track_name = track_publisher->trackName();
  const auto message = makeBatteryState();

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, message, room_connection, 1U));

  track_publisher.reset();

  EXPECT_EQ(room_connection.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  publishAndDrain(executor, publisher, message, std::chrono::milliseconds(60));
  EXPECT_EQ(room_connection.state->pushed_data_track_frames.size(), 1U);
}

TEST(DataTrackPublisherTest, DestructionSwallowsUnpublishFailure)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_unpublish_failure_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/per_track_unpublish_failure";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, batteryStateInterfaceType()));

  auto track_publisher = createDataTrackPublisher(*node, room_connection, topic);
  track_publisher->publish();
  const std::string track_name = track_publisher->trackName();
  room_connection.state->unpublish_rejected_data_track_names.push_back(track_name);

  ASSERT_TRUE(publishUntilFrameCount(executor, publisher, makeBatteryState(), room_connection, 1U));

  track_publisher.reset();

  EXPECT_EQ(room_connection.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

}  // namespace
}  // namespace livekit_ros2_bridge
