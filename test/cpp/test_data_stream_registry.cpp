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
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "data_stream_registry.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
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

TEST(DataStreamRegistryTest, FailureCallbackUsesTrackNameLookup)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_registry_failure_callback_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/data_registry_failure_callback";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry registry(*node, room_connection);
  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);

  const auto * data = registry.find(topic);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(data->state(), DataStreamInstance::State::kPublished);

  registry.onTrackFailed(data->trackName());
  EXPECT_EQ(data->state(), DataStreamInstance::State::kFailed);
}

TEST(DataStreamRegistryTest, StopAdvancesGenerationAndAllowsRecreate)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_registry_shutdown_handle_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/data_registry_shutdown_handle";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry registry(*node, room_connection);
  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);
  const auto * first = registry.find(topic);
  ASSERT_NE(first, nullptr);
  const std::string first_track_name = first->trackName();

  const auto generation_before_shutdown = registry.generation();
  registry.stop(topic);

  EXPECT_GT(registry.generation(), generation_before_shutdown);
  EXPECT_EQ(room_connection.state->unpublished_data_track_names, std::vector<std::string>{first_track_name});

  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);
  EXPECT_EQ(
    room_connection.state->published_data_track_names, (std::vector<std::string>{first_track_name, first_track_name}));
}

TEST(DataStreamRegistryTest, ResetSessionStateRejectsLatePublishCompletionAndAllowsRecreate)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_registry_reset_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/data_registry_reset";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  DataStreamRegistry registry(*node, room_connection);
  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);
  const auto * first = registry.find(topic);
  ASSERT_NE(first, nullptr);
  const std::string first_track_name = first->trackName();

  const auto stale_generation = registry.generation();
  registry.resetSessionState();

  EXPECT_FALSE(registry.onTrackPublished(first_track_name, stale_generation));
  EXPECT_EQ(room_connection.state->unpublished_data_track_names, std::vector<std::string>{first_track_name});

  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);
  EXPECT_EQ(
    room_connection.state->published_data_track_names, (std::vector<std::string>{first_track_name, first_track_name}));
}

TEST(DataStreamRegistryTest, ShutdownWaitsForActiveSerializedMessageCallback)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_registry_shutdown_quiesce_test");
  FakeRoomConnection room_connection;
  const std::string topic = "/battery/data_registry_shutdown_quiesce";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  auto push_entered = std::make_shared<std::promise<void>>();
  auto push_entered_future = push_entered->get_future();
  auto release_push = std::make_shared<std::promise<void>>();
  auto release_push_future = release_push->get_future().share();
  room_connection.state->try_push_data_track_handler = [push_entered, release_push_future](
                                                         const std::string &, const std::vector<std::uint8_t> &) {
    push_entered->set_value();
    release_push_future.wait();
    return DataTrackPushResult::success();
  };

  DataStreamRegistry registry(*node, room_connection);
  registry.create(topic, "sensor_msgs/msg/BatteryState");
  registry.start(topic);
  const auto * data = registry.find(topic);
  ASSERT_NE(data, nullptr);
  const std::string track_name = data->trackName();

  std::thread spin_thread([&executor]() { executor.spin(); });

  publisher->publish(makeBatteryState());
  EXPECT_EQ(push_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto shutdown_future = std::async(std::launch::async, [&registry]() { registry.shutdown(); });
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  release_push->set_value();

  EXPECT_EQ(shutdown_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  shutdown_future.get();
  EXPECT_EQ(room_connection.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  executor.cancel();
  spin_thread.join();
}

}  // namespace
}  // namespace livekit_ros2_bridge
