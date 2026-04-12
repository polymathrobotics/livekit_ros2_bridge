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
#include <utility>
#include <vector>

#include "data_stream_instance.hpp"
#include "gtest/gtest.h"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "utils/quiesce_guard.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;
using test_support::waitForTopicType;

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

TEST(DataStreamInstanceTest, CreatesBestEffortSubscriptionAndPublishesPendingTrackOnStart)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_best_effort_test");
  const std::string topic = "/battery/best_effort_instance";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10).best_effort());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  QuiesceGate message_callback_gate;
  std::vector<CdrFrame> cdr_frames;
  std::vector<std::pair<std::string, std::size_t>> publish_requests;
  auto instance = DataStreamInstance::create(
    *node,
    topic,
    "sensor_msgs/msg/BatteryState",
    0,
    7,
    message_callback_gate,
    [&cdr_frames](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({track_name, std::vector<std::uint8_t>(data, data + size)});
    },
    [&publish_requests](const std::string & track_name, std::size_t generation) {
      publish_requests.emplace_back(track_name, generation);
    },
    [](const std::string &) {});

  instance->start("alice");

  ASSERT_EQ(publish_requests.size(), 1U);
  EXPECT_EQ(publish_requests[0].first, "ros.data.battery.best_effort_instance");
  EXPECT_EQ(publish_requests[0].second, 7U);
  EXPECT_EQ(instance->trackName(), "ros.data.battery.best_effort_instance");
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kPending);

  const auto message = makeBatteryState();
  ASSERT_TRUE(publishUntil(executor, publisher, message, [&]() { return cdr_frames.size() == 1U; }));
  EXPECT_EQ(cdr_frames[0].track_name, instance->trackName());
  EXPECT_EQ(deserializeMessage<sensor_msgs::msg::BatteryState>(cdr_frames[0].data), message);
}

TEST(DataStreamInstanceTest, SendsSerializedMessagesOnlyWhenPendingOrPublished)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_delivery_state_test");
  const std::string topic = "/battery/delivery_state";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  QuiesceGate message_callback_gate;
  std::vector<CdrFrame> cdr_frames;
  std::vector<std::pair<std::string, std::size_t>> publish_requests;
  auto instance = DataStreamInstance::create(
    *node,
    topic,
    "sensor_msgs/msg/BatteryState",
    0,
    11,
    message_callback_gate,
    [&cdr_frames](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({track_name, std::vector<std::uint8_t>(data, data + size)});
    },
    [&publish_requests](const std::string & track_name, std::size_t generation) {
      publish_requests.emplace_back(track_name, generation);
    },
    [](const std::string &) {});

  publisher->publish(makeBatteryState());
  executor.spin_some();
  EXPECT_TRUE(cdr_frames.empty());

  instance->start("alice");
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kPending);

  EXPECT_FALSE(instance->onPublishComplete(10));
  EXPECT_TRUE(instance->onPublishComplete(11));
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 2U; }));
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kPublished);

  instance->onPublishFailed();
  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(cdr_frames.size(), 2U);
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kFailed);

  instance->start("alice");
  ASSERT_EQ(publish_requests.size(), 2U);
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 3U; }));
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kPending);
}

TEST(DataStreamInstanceTest, SuppressesMessagesAccordingToAppliedInterval)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_interval_test");
  const std::string topic = "/battery/interval_instance";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  QuiesceGate message_callback_gate;
  std::vector<CdrFrame> cdr_frames;
  auto instance = DataStreamInstance::create(
    *node,
    topic,
    "sensor_msgs/msg/BatteryState",
    150,
    5,
    message_callback_gate,
    [&cdr_frames](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({track_name, std::vector<std::uint8_t>(data, data + size)});
    },
    [](const std::string &, std::size_t) {},
    [](const std::string &) {});

  instance->start("alice");

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));

  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(cdr_frames.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 2U; }));

  instance->updateAppliedIntervalMs(0);
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 3U; }));
}

TEST(DataStreamInstanceTest, RepublishResetsSuppressionAndRequestsNewPublish)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_republish_test");
  const std::string topic = "/battery/republish_instance";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  QuiesceGate message_callback_gate;
  std::vector<CdrFrame> cdr_frames;
  std::vector<std::pair<std::string, std::size_t>> publish_requests;
  std::vector<std::string> unpublished_track_names;
  auto instance = DataStreamInstance::create(
    *node,
    topic,
    "sensor_msgs/msg/BatteryState",
    1000,
    19,
    message_callback_gate,
    [&cdr_frames](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({track_name, std::vector<std::uint8_t>(data, data + size)});
    },
    [&publish_requests](const std::string & track_name, std::size_t generation) {
      publish_requests.emplace_back(track_name, generation);
    },
    [&unpublished_track_names](const std::string & track_name) { unpublished_track_names.push_back(track_name); });

  instance->start("alice");
  ASSERT_TRUE(instance->onPublishComplete(19));
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));

  instance->republish("alice");

  ASSERT_EQ(unpublished_track_names.size(), 1U);
  EXPECT_EQ(unpublished_track_names[0], instance->trackName());
  ASSERT_EQ(publish_requests.size(), 2U);
  EXPECT_EQ(publish_requests[0], publish_requests[1]);
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kPending);

  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 2U; }));
}

TEST(DataStreamInstanceTest, ShutdownUnpublishesPublishedTrackAndDropsSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_stream_instance_shutdown_test");
  const std::string topic = "/battery/shutdown_instance";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  QuiesceGate message_callback_gate;
  std::vector<CdrFrame> cdr_frames;
  std::vector<std::string> unpublished_track_names;
  auto instance = DataStreamInstance::create(
    *node,
    topic,
    "sensor_msgs/msg/BatteryState",
    0,
    23,
    message_callback_gate,
    [&cdr_frames](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_frames.push_back({track_name, std::vector<std::uint8_t>(data, data + size)});
    },
    [](const std::string &, std::size_t) {},
    [&unpublished_track_names](const std::string & track_name) { unpublished_track_names.push_back(track_name); });

  instance->start("alice");
  ASSERT_TRUE(instance->onPublishComplete(23));
  ASSERT_TRUE(publishUntil(executor, publisher, makeBatteryState(), [&]() { return cdr_frames.size() == 1U; }));

  instance->shutdown();

  ASSERT_EQ(unpublished_track_names.size(), 1U);
  EXPECT_EQ(unpublished_track_names[0], instance->trackName());
  EXPECT_EQ(instance->state(), DataStreamInstance::State::kNone);

  publisher->publish(makeBatteryState());
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  executor.spin_some();
  EXPECT_EQ(cdr_frames.size(), 1U);

  instance->shutdown();
  EXPECT_EQ(unpublished_track_names.size(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
