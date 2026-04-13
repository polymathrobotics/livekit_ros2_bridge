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
#include <map>
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

#define private public
#include "ros_topic_publisher.hpp"
#undef private

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename MessageT>
std::vector<std::uint8_t> serializeMessageToCdr(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

TopicPublishCommand makeCommand(
  const std::string & topic, const std::string & interface_type, std::vector<std::uint8_t> cdr)
{
  TopicPublishCommand command;
  command.topic = topic;
  command.interface_type = interface_type;
  command.cdr = std::move(cdr);
  return command;
}

AccessPolicy makeAccessPolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.publish.allow = std::move(allow);
  config.publish.deny = std::move(deny);
  return AccessPolicy(config);
}

class RosTopicPublisherHarness final
{
public:
  RosTopicPublisherHarness()
  {
    observer_context_ = std::make_shared<rclcpp::Context>();
    observer_context_->init(0, nullptr);

    rclcpp::NodeOptions observer_options;
    observer_options.context(observer_context_);
    observer_node_ = std::make_shared<rclcpp::Node>(nextNodeName("topic_publisher_observer"), observer_options);

    rclcpp::ExecutorOptions observer_executor_options;
    observer_executor_options.context = observer_context_;
    observer_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(observer_executor_options);
    observer_executor_->add_node(observer_node_);

    resetPublisher("topic_publisher_node");
  }

  ~RosTopicPublisherHarness()
  {
    publisher_executor_.reset();
    publisher_node_.reset();
    if (publisher_context_ && publisher_context_->is_valid()) {
      publisher_context_->shutdown("topic publisher test teardown");
    }
    publisher_context_.reset();

    observer_executor_.reset();
    observer_node_.reset();
    if (observer_context_ && observer_context_->is_valid()) {
      observer_context_->shutdown("topic publisher test teardown");
    }
    observer_context_.reset();
  }

  void resetPublisher(const std::string & node_prefix)
  {
    publisher_executor_.reset();
    publisher_node_.reset();
    if (publisher_context_ && publisher_context_->is_valid()) {
      publisher_context_->shutdown("topic publisher test reset");
    }

    publisher_context_ = std::make_shared<rclcpp::Context>();
    publisher_context_->init(0, nullptr);

    rclcpp::NodeOptions publisher_options;
    publisher_options.context(publisher_context_);
    publisher_node_ = std::make_shared<rclcpp::Node>(nextNodeName(node_prefix), publisher_options);

    rclcpp::ExecutorOptions publisher_executor_options;
    publisher_executor_options.context = publisher_context_;
    publisher_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(publisher_executor_options);
    publisher_executor_->add_node(publisher_node_);
  }

  rclcpp::Node & publisherNode() const
  {
    return *publisher_node_;
  }

  rclcpp::Node & observerNode() const
  {
    return *observer_node_;
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

  bool waitForType(
    const std::string & topic,
    const std::string & expected_type,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    return spinUntil(
      [&]() {
        const auto topics = publisher_node_->get_topic_names_and_types();
        const auto topic_it = topics.find(topic);
        return topic_it != topics.end() && topic_it->second.size() == 1U && topic_it->second.front() == expected_type;
      },
      timeout);
  }

  std::size_t countPublishers(const std::string & topic)
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
    if (publisher_executor_ && publisher_context_ && publisher_context_->is_valid()) {
      publisher_executor_->spin_some();
    }
    if (observer_executor_ && observer_context_ && observer_context_->is_valid()) {
      observer_executor_->spin_some();
    }
  }

  std::shared_ptr<rclcpp::Context> publisher_context_;
  std::shared_ptr<rclcpp::Context> observer_context_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> observer_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> publisher_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> observer_executor_;
};

void expectTopicNotPublished(
  RosTopicPublisherHarness & harness,
  const std::string & topic,
  const std::optional<sensor_msgs::msg::BatteryState> & received_message)
{
  EXPECT_FALSE(harness.spinUntil([&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.countPublishers(topic), 0U);
}

TEST(TopicPublisherTest, PublishesMessagesToCommandTopic)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/cmd";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
}

TEST(TopicPublisherTest, RejectsDeniedPublishCommands)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/blocked";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({"/battery/allowed"}));

  sensor_msgs::msg::BatteryState message;

  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, CacheSizeOneEvictsPreviousTopicAndRecreatesItOnReuse)
{
  RosTopicPublisherHarness harness;
  const std::string first_topic = "/battery/first";
  const std::string second_topic = "/battery/second";

  std::vector<float> first_topic_voltages;
  std::vector<float> second_topic_voltages;
  [[maybe_unused]] const auto first_subscription =
    harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
      first_topic, rclcpp::QoS(10), [&first_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
        first_topic_voltages.push_back(message.voltage);
      });
  [[maybe_unused]] const auto second_subscription =
    harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
      second_topic, rclcpp::QoS(10), [&second_topic_voltages](const sensor_msgs::msg::BatteryState & message) {
        second_topic_voltages.push_back(message.voltage);
      });

  ASSERT_TRUE(harness.waitForType(first_topic, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(harness.waitForType(second_topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({"/battery/*"}), 1U);

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  publisher.publish("alice", makeCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 48.5F, 1e-6F);
  EXPECT_EQ(harness.countPublishers(first_topic), 1U);

  message.voltage = 49.0F;
  publisher.publish("alice", makeCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 2U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 49.0F, 1e-6F);

  message.voltage = 47.0F;
  publisher.publish("alice", makeCommand(second_topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return second_topic_voltages.size() == 1U; }));
  EXPECT_NEAR(second_topic_voltages.back(), 47.0F, 1e-6F);
  ASSERT_TRUE(harness.spinUntil(
    [&]() { return harness.countPublishers(first_topic) == 0U && harness.countPublishers(second_topic) == 1U; }));

  message.voltage = 50.0F;
  publisher.publish("alice", makeCommand(first_topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return first_topic_voltages.size() == 3U; }));
  EXPECT_NEAR(first_topic_voltages.back(), 50.0F, 1e-6F);
  ASSERT_TRUE(harness.spinUntil(
    [&]() { return harness.countPublishers(first_topic) == 1U && harness.countPublishers(second_topic) == 0U; }));
}

TEST(TopicPublisherTest, CachedPublisherSkipsGraphLookupAfterFirstSuccessfulPublish)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/cached_type";

  std::vector<float> received_voltages;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_voltages](const sensor_msgs::msg::BatteryState & message) {
      received_voltages.push_back(message.voltage);
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(first_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_voltages.size() == 1U; }));
  EXPECT_NEAR(received_voltages.back(), 48.5F, 1e-6F);

  std::size_t graph_calls = 0U;
  publisher.topic_graph_provider_ = [&graph_calls]() {
    ++graph_calls;
    return std::map<std::string, std::vector<std::string>>{};
  };

  sensor_msgs::msg::BatteryState second_message;
  second_message.voltage = 49.0F;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(second_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_voltages.size() == 2U; }));
  EXPECT_NEAR(received_voltages.back(), 49.0F, 1e-6F);
  EXPECT_EQ(graph_calls, 0U);
  EXPECT_EQ(harness.countPublishers(topic), 1U);
}

TEST(TopicPublisherTest, CachedPublisherRejectsMismatchedDeclaredTypeWithoutGraphLookup)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/cached_type_mismatch";

  std::vector<float> received_voltages;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_voltages](const sensor_msgs::msg::BatteryState & message) {
      received_voltages.push_back(message.voltage);
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(first_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_voltages.size() == 1U; }));
  EXPECT_NEAR(received_voltages.back(), 48.5F, 1e-6F);

  std::size_t graph_calls = 0U;
  publisher.topic_graph_provider_ = [&graph_calls]() {
    ++graph_calls;
    return std::map<std::string, std::vector<std::string>>{};
  };

  std_msgs::msg::String wrong_message;
  wrong_message.data = "wrong type";
  publisher.publish("alice", makeCommand(topic, "std_msgs/msg/String", serializeMessageToCdr(wrong_message)));

  EXPECT_FALSE(harness.spinUntil([&]() { return received_voltages.size() > 1U; }, std::chrono::milliseconds(200)));
  EXPECT_EQ(graph_calls, 0U);
  EXPECT_EQ(harness.countPublishers(topic), 1U);
}

TEST(TopicPublisherTest, FailedFirstPublishDoesNotLeavePublisherRegisteredAndLaterPublishStillSucceeds)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/failure_cleanup";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState", std::chrono::seconds(5)));

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  const TopicPublishCommand command =
    makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message));

  bool fail_first_publish = true;
  bool subscriber_ready = false;
  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));
  publisher.before_publish_handler_ = [&]() {
    subscriber_ready = harness.spinUntil([&]() { return harness.publisherNode().count_subscribers(topic) == 1U; });
    if (fail_first_publish) {
      fail_first_publish = false;
      throw std::runtime_error("forced publish failure");
    }
  };

  publisher.publish("alice", command);

  EXPECT_TRUE(subscriber_ready);
  expectTopicNotPublished(harness, topic, received_message);

  publisher.publish("alice", command);

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
}

TEST(TopicPublisherTest, RejectsCommandsWhoseDeclaredTypeDoesNotMatchTheGraph)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/invalid";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));

  std_msgs::msg::String wrong_message;

  publisher.publish("alice", makeCommand(topic, "std_msgs/msg/String", serializeMessageToCdr(wrong_message)));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, RejectsCommandsForTopicsMissingFromTheGraph)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/missing";

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));

  sensor_msgs::msg::BatteryState message;

  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  EXPECT_FALSE(harness.spinUntil(
    [&]() { return harness.observerNode().count_publishers(topic) != 0U; }, std::chrono::milliseconds(200)));
}

TEST(TopicPublisherTest, RejectsCommandsWhenTopicGraphHasMultipleTypes)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/ambiguous";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto battery_subscription =
    harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
      topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
        received_message = message;
      });

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));
  publisher.topic_graph_provider_ = [topic]() {
    return std::map<std::string, std::vector<std::string>>{
      {topic, {"sensor_msgs/msg/BatteryState", "std_msgs/msg/String"}},
    };
  };

  sensor_msgs::msg::BatteryState message;

  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(message)));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, ShutdownPreventsRosPublisherRecreationAndRepeatedShutdownIsHarmless)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/shutdown_terminal";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(first_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_EQ(harness.countPublishers(topic), 1U);

  publisher.shutdown();
  publisher.shutdown();

  ASSERT_TRUE(harness.spinUntil([&]() { return harness.countPublishers(topic) == 0U; }));

  received_message.reset();
  sensor_msgs::msg::BatteryState late_message;

  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(late_message)));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, ShutdownDuringInFlightFirstPublishDoesNotLeavePublisherRegisteredOrRecreateIt)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/shutdown_race";

  std::atomic<std::size_t> delivered_messages{0U};
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&delivered_messages](const sensor_msgs::msg::BatteryState &) {
      delivered_messages.fetch_add(1U);
    });

  ASSERT_TRUE(harness.waitForType(topic, "sensor_msgs/msg/BatteryState"));

  RosTopicPublisher publisher(harness.publisherNode(), makeAccessPolicy({topic}));
  publisher.before_publish_handler_ = [&]() {
    EXPECT_TRUE(harness.spinUntil([&]() { return harness.publisherNode().count_subscribers(topic) == 1U; }));
    publisher.shutdown();
  };

  sensor_msgs::msg::BatteryState first_message;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(first_message)));

  ASSERT_TRUE(harness.spinUntil([&]() { return harness.countPublishers(topic) == 0U; }));

  const std::size_t deliveries_after_shutdown = delivered_messages.load();

  sensor_msgs::msg::BatteryState second_message;
  publisher.publish("alice", makeCommand(topic, "sensor_msgs/msg/BatteryState", serializeMessageToCdr(second_message)));

  EXPECT_FALSE(harness.spinUntil(
    [&]() { return delivered_messages.load() > deliveries_after_shutdown; }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.countPublishers(topic), 0U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
