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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#define private public
#include "ros_topic_writer.hpp"
#undef private

namespace livekit_ros2_bridge
{
namespace
{

template <typename MessageT>
std::vector<std::uint8_t> serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

TopicPublishCommand makePublishCommand(
  const std::string & topic, const std::string & interface_type, std::vector<std::uint8_t> cdr_payload)
{
  TopicPublishCommand command;
  command.topic = topic;
  command.interface_type = interface_type;
  command.cdr_payload = std::move(cdr_payload);
  return command;
}

AccessPolicy makePublishPolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.publish.allow = std::move(allow);
  config.publish.deny = std::move(deny);
  return AccessPolicy(config);
}

class TopicWriterHarness final
{
public:
  TopicWriterHarness()
  {
    observer_context_ = std::make_shared<rclcpp::Context>();
    observer_context_->init(0, nullptr);

    rclcpp::NodeOptions observer_options;
    observer_options.context(observer_context_);
    observer_node_ = std::make_shared<rclcpp::Node>(nextNodeName("topic_writer_observer"), observer_options);

    rclcpp::ExecutorOptions observer_executor_options;
    observer_executor_options.context = observer_context_;
    observer_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(observer_executor_options);
    observer_executor_->add_node(observer_node_);

    recreateWriterSide("topic_writer_node");
  }

  ~TopicWriterHarness()
  {
    writer_executor_.reset();
    writer_node_.reset();
    if (writer_context_ && writer_context_->is_valid()) {
      writer_context_->shutdown("topic writer test teardown");
    }
    writer_context_.reset();

    observer_executor_.reset();
    observer_node_.reset();
    if (observer_context_ && observer_context_->is_valid()) {
      observer_context_->shutdown("topic writer test teardown");
    }
    observer_context_.reset();
  }

  void recreateWriterSide(const std::string & node_prefix)
  {
    writer_executor_.reset();
    writer_node_.reset();
    if (writer_context_ && writer_context_->is_valid()) {
      writer_context_->shutdown("topic writer test reset");
    }

    writer_context_ = std::make_shared<rclcpp::Context>();
    writer_context_->init(0, nullptr);

    rclcpp::NodeOptions writer_options;
    writer_options.context(writer_context_);
    writer_node_ = std::make_shared<rclcpp::Node>(nextNodeName(node_prefix), writer_options);

    rclcpp::ExecutorOptions writer_executor_options;
    writer_executor_options.context = writer_context_;
    writer_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(writer_executor_options);
    writer_executor_->add_node(writer_node_);
  }

  rclcpp::Node & writerNode() const
  {
    return *writer_node_;
  }

  rclcpp::Node & observerNode() const
  {
    return *observer_node_;
  }

  void shutdownWriterContext()
  {
    if (writer_context_ && writer_context_->is_valid()) {
      writer_context_->shutdown("topic writer test induced shutdown");
    }
  }

  bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      spinSome();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    spinSome();
    return predicate();
  }

  bool waitForTopicType(
    const std::string & topic,
    const std::string & expected_type,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    return spinUntil(
      [&]() {
        const auto topics = writer_node_->get_topic_names_and_types();
        const auto topic_it = topics.find(topic);
        return topic_it != topics.end() && topic_it->second.size() == 1U && topic_it->second.front() == expected_type;
      },
      timeout);
  }

  std::size_t publisherCount(const std::string & topic)
  {
    spinSome();
    return observer_node_->count_publishers(topic);
  }

private:
  static std::string nextNodeName(const std::string & prefix)
  {
    static std::atomic<std::size_t> counter{0U};
    return prefix + "_" + std::to_string(counter.fetch_add(1U));
  }

  void spinSome()
  {
    if (writer_executor_ && writer_context_ && writer_context_->is_valid()) {
      writer_executor_->spin_some();
    }
    if (observer_executor_ && observer_context_ && observer_context_->is_valid()) {
      observer_executor_->spin_some();
    }
  }

  std::shared_ptr<rclcpp::Context> writer_context_;
  std::shared_ptr<rclcpp::Context> observer_context_;
  std::shared_ptr<rclcpp::Node> writer_node_;
  std::shared_ptr<rclcpp::Node> observer_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> writer_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> observer_executor_;
};

TEST(TopicWriterTest, WritesMessagesToCommandTopic)
{
  TopicWriterHarness harness;
  const std::string topic = "/battery/cmd";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({topic}));
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  writer.write("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
  EXPECT_EQ(harness.publisherCount(topic), 1U);
}

TEST(TopicWriterTest, RejectsDeniedPublishCommands)
{
  TopicWriterHarness harness;
  const std::string topic = "/battery/blocked";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({"/battery/allowed"}));

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;

  writer.write("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  EXPECT_FALSE(harness.spinUntil([&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.publisherCount(topic), 0U);
}

TEST(TopicWriterTest, ReusesPublishersAndEvictsLeastRecentlyUsedTopic)
{
  TopicWriterHarness harness;
  const std::string first_topic = "/battery/first";
  const std::string second_topic = "/battery/second";

  std::vector<float> first_topic_voltages;
  std::vector<float> second_topic_voltages;
  auto first_subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    first_topic, rclcpp::QoS(10), [&first_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
      first_topic_voltages.push_back(message.voltage);
    });
  auto second_subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    second_topic, rclcpp::QoS(10), [&second_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
      second_topic_voltages.push_back(message.voltage);
    });
  ASSERT_NE(first_subscription, nullptr);
  ASSERT_NE(second_subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(first_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(harness.waitForTopicType(second_topic, "sensor_msgs/msg/BatteryState"));

  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({"/battery/*"}), 1U);

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  writer.write("alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 48.5F, 1e-6F);
  EXPECT_EQ(harness.publisherCount(first_topic), 1U);
  EXPECT_EQ(harness.publisherCount(second_topic), 0U);

  message.voltage = 49.0F;
  writer.write("alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 2U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 49.0F, 1e-6F);
  EXPECT_TRUE(second_topic_voltages.empty());
  EXPECT_EQ(harness.publisherCount(first_topic), 1U);
  EXPECT_EQ(harness.publisherCount(second_topic), 0U);

  message.voltage = 47.0F;
  writer.write("alice", makePublishCommand(second_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return second_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(second_topic_voltages.back(), 47.0F, 1e-6F);
  ASSERT_TRUE(harness.spinUntil(
    [&]() { return harness.publisherCount(first_topic) == 0U && harness.publisherCount(second_topic) == 1U; }));

  message.voltage = 50.0F;
  writer.write("alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 3U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 50.0F, 1e-6F);
  ASSERT_TRUE(harness.spinUntil(
    [&]() { return harness.publisherCount(first_topic) == 1U && harness.publisherCount(second_topic) == 0U; }));
}

TEST(TopicWriterTest, FailedFirstWriteDoesNotLeavePublisherRegisteredAndLaterWriteStillSucceeds)
{
  TopicWriterHarness harness;
  const std::string topic = "/battery/failure_cleanup";
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(topic, "sensor_msgs/msg/BatteryState", std::chrono::seconds(5)));

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  const TopicPublishCommand command =
    makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message));

  bool force_first_write_failure = true;
  bool subscriber_ready_before_write = false;
  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({topic}));
  writer.setBeforeWriteHookForTest([&]() {
    subscriber_ready_before_write =
      harness.spinUntil([&]() { return harness.writerNode().count_subscribers(topic) == 1U; });
    if (force_first_write_failure) {
      force_first_write_failure = false;
      throw std::runtime_error("forced publish failure");
    }
  });

  writer.write("alice", command);

  EXPECT_TRUE(subscriber_ready_before_write);
  EXPECT_FALSE(received_message.has_value());
  EXPECT_TRUE(writer.cached_publishers_.empty());

  subscriber_ready_before_write = false;
  writer.write("alice", command);

  EXPECT_TRUE(subscriber_ready_before_write);
  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  ASSERT_EQ(writer.cached_publishers_.size(), 1U);
  ASSERT_TRUE(writer.cached_publishers_.peek(topic).has_value());
  EXPECT_EQ(writer.cached_publishers_.peek(topic)->interface_type, "sensor_msgs/msg/BatteryState");
}

TEST(TopicWriterTest, RejectsCommandsWhoseDeclaredTypeDoesNotMatchTheGraph)
{
  TopicWriterHarness harness;
  const std::string topic = "/battery/invalid";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({topic}));

  std_msgs::msg::String wrong_message;
  wrong_message.data = "not a BatteryState";

  writer.write("alice", makePublishCommand(topic, "std_msgs/msg/String", serializeMessage(wrong_message)));

  EXPECT_FALSE(harness.spinUntil([&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.publisherCount(topic), 0U);
}

TEST(TopicWriterTest, ShutdownPreventsRosPublisherRecreationAndRepeatedShutdownIsHarmless)
{
  TopicWriterHarness harness;
  const std::string topic = "/battery/shutdown_terminal";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

  ASSERT_TRUE(harness.waitForTopicType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicWriter writer(harness.writerNode(), makePublishPolicy({topic}));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  writer.write("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(first_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_EQ(harness.publisherCount(topic), 1U);

  writer.shutdown();
  writer.shutdown();

  ASSERT_TRUE(harness.spinUntil([&]() { return harness.publisherCount(topic) == 0U; }));

  received_message.reset();
  sensor_msgs::msg::BatteryState late_message;
  late_message.voltage = 49.0F;

  writer.write("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(late_message)));

  EXPECT_FALSE(harness.spinUntil([&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.publisherCount(topic), 0U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
