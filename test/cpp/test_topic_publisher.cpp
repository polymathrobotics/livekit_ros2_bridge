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
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "topic_publisher.hpp"

namespace livekit_ros2_bridge
{

class RosTopicPublisherTestPeer final
{
public:
  static void setBeforePublishHook(RosTopicPublisher & publisher, std::function<void()> hook)
  {
    publisher.setBeforePublishHookForTest(std::move(hook));
  }

  static bool cacheAndLruAreAligned(const RosTopicPublisher & publisher)
  {
    if (publisher.publishers_.size() != publisher.lru_topics_.size()) {
      return false;
    }

    for (auto lru_it = publisher.lru_topics_.begin(); lru_it != publisher.lru_topics_.end(); ++lru_it) {
      const auto publisher_it = publisher.publishers_.find(*lru_it);
      if (publisher_it == publisher.publishers_.end()) {
        return false;
      }
      if (publisher_it->second.lru_position != lru_it) {
        return false;
      }
    }

    for (const auto & entry : publisher.publishers_) {
      if (
        std::find(publisher.lru_topics_.begin(), publisher.lru_topics_.end(), entry.first) ==
        publisher.lru_topics_.end())
      {
        return false;
      }
    }

    return true;
  }

  static std::size_t cachedTopicCount(const RosTopicPublisher & publisher)
  {
    return publisher.publishers_.size();
  }
};

namespace
{

class ScopedRclcppInit
{
public:
  ScopedRclcppInit()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~ScopedRclcppInit()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

bool waitForTopicType(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic,
  const std::string & expected_type)
{
  return spinUntil(executor, [&]() {
    const auto topics = node->get_topic_names_and_types();
    for (const auto & entry : topics) {
      if (entry.first == topic && entry.second.size() == 1U && entry.second.front() == expected_type) {
        return true;
      }
    }
    return false;
  });
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

TopicPublishCommand makePublishCommand(
  const std::string & topic, const std::string & interface_type, std::vector<std::uint8_t> cdr_payload)
{
  TopicPublishCommand command;
  command.topic = topic;
  command.interface_type = interface_type;
  command.cdr_payload = std::move(cdr_payload);
  return command;
}

TEST(TopicPublisherTest, PublishesMessagesToCommandTopic)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_observer_node");
  const std::string topic = "/battery/cmd";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({topic}, {}, {}, {}, {}, {}), 50);

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  const TopicPublishCommand command =
    makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message));
  publisher.publish("alice", command);

  ASSERT_TRUE(spinUntil(executor, [&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
}

TEST(TopicPublisherTest, ConstructorRejectsNegativeMaxTopicsAndAcceptsUnlimitedZero)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_constructor_publisher_node");
  const auto empty_access_policy = AccessPolicy({}, {}, {}, {}, {}, {});

  EXPECT_THROW(RosTopicPublisher(*publisher_node, empty_access_policy, -1), std::invalid_argument);
  EXPECT_NO_THROW(RosTopicPublisher(*publisher_node, empty_access_policy, 0));
}

TEST(TopicPublisherTest, RejectsDeniedPublishCommands)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_denied_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_denied_observer_node");
  const std::string topic = "/battery/blocked";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({"/battery/allowed"}, {}, {}, {}, {}, {}), 50);

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;

  publisher.publish("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  EXPECT_FALSE(spinUntil(executor, [&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
}

TEST(TopicPublisherTest, ReusesPublishersAndEvictsLeastRecentlyUsedTopic)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_lru_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_lru_observer_node");
  const std::string first_topic = "/battery/first";
  const std::string second_topic = "/battery/second";

  std::vector<float> first_topic_voltages;
  std::vector<float> second_topic_voltages;
  auto first_subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    first_topic, rclcpp::QoS(10), [&first_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
      first_topic_voltages.push_back(message.voltage);
    });
  auto second_subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    second_topic, rclcpp::QoS(10), [&second_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
      second_topic_voltages.push_back(message.voltage);
    });
  (void)first_subscription;
  (void)second_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, first_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, second_topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({"/battery/*"}, {}, {}, {}, {}, {}), 1);

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;

  publisher.publish(
    "alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));
  ASSERT_TRUE(spinUntil(executor, [&]() { return first_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 48.5F, 1e-6F);
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));

  message.voltage = 49.0F;
  publisher.publish(
    "alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));
  ASSERT_TRUE(spinUntil(executor, [&]() { return first_topic_voltages.size() == 2U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 49.0F, 1e-6F);
  EXPECT_TRUE(second_topic_voltages.empty());
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));

  message.voltage = 47.0F;
  publisher.publish(
    "alice", makePublishCommand(second_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));
  ASSERT_TRUE(spinUntil(executor, [&]() { return second_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(second_topic_voltages.back(), 47.0F, 1e-6F);
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));
  ASSERT_TRUE(spinUntil(executor, [&]() {
    return observer_node->count_publishers(first_topic) == 0U && observer_node->count_publishers(second_topic) == 1U;
  }));

  message.voltage = 50.0F;
  publisher.publish(
    "alice", makePublishCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));
  ASSERT_TRUE(spinUntil(executor, [&]() { return first_topic_voltages.size() == 3U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 50.0F, 1e-6F);
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));
  ASSERT_TRUE(spinUntil(executor, [&]() {
    return observer_node->count_publishers(first_topic) == 1U && observer_node->count_publishers(second_topic) == 0U;
  }));
}

TEST(TopicPublisherTest, FailedFirstPublishDoesNotLeavePartialCacheEntryBehind)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_failure_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_failure_observer_node");
  const std::string topic = "/battery/failure_cleanup";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({topic}, {}, {}, {}, {}, {}), 50);

  RosTopicPublisherTestPeer::setBeforePublishHook(
    publisher, []() { throw std::runtime_error("simulated publish failure"); });
  publisher.publish(
    "alice",
    makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(sensor_msgs::msg::BatteryState{})));

  EXPECT_EQ(RosTopicPublisherTestPeer::cachedTopicCount(publisher), 0U);
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));
  EXPECT_TRUE(spinUntil(
    executor, [&]() { return observer_node->count_publishers(topic) == 0U; }, std::chrono::milliseconds(200)));

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  RosTopicPublisherTestPeer::setBeforePublishHook(publisher, {});

  publisher.publish("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(message)));

  ASSERT_TRUE(spinUntil(executor, [&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_EQ(RosTopicPublisherTestPeer::cachedTopicCount(publisher), 1U);
  EXPECT_TRUE(RosTopicPublisherTestPeer::cacheAndLruAreAligned(publisher));
}

TEST(TopicPublisherTest, RejectsCommandsWhoseDeclaredTypeDoesNotMatchTheGraph)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_adapter_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_adapter_observer_node");
  const std::string topic = "/battery/invalid";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({topic}, {}, {}, {}, {}, {}), 50);

  std_msgs::msg::String wrong_message;
  wrong_message.data = "not a BatteryState";

  publisher.publish("alice", makePublishCommand(topic, "std_msgs/msg/String", serializeMessage(wrong_message)));

  EXPECT_FALSE(spinUntil(executor, [&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
}

TEST(TopicPublisherTest, ShutdownPreventsPublisherRecreationAndRepeatedShutdownIsHarmless)
{
  ScopedRclcppInit init;
  auto publisher_node = std::make_shared<rclcpp::Node>("topic_publish_shutdown_publisher_node");
  auto observer_node = std::make_shared<rclcpp::Node>("topic_publish_shutdown_observer_node");
  const std::string topic = "/battery/shutdown_terminal";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer_node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(observer_node);
  ASSERT_TRUE(waitForTopicType(executor, publisher_node, topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(*publisher_node, AccessPolicy({topic}, {}, {}, {}, {}, {}), 50);

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;

  publisher.publish(
    "alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(first_message)));

  ASSERT_TRUE(spinUntil(executor, [&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);

  publisher.shutdown();
  publisher.shutdown();

  received_message.reset();
  sensor_msgs::msg::BatteryState late_message;
  late_message.voltage = 49.0F;

  publisher.publish("alice", makePublishCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessage(late_message)));

  EXPECT_FALSE(spinUntil(executor, [&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
}

}  // namespace
}  // namespace livekit_ros2_bridge
