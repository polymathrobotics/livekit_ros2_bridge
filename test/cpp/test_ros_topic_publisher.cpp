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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_topic_publisher.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename MessageT>
rclcpp::SerializedMessage serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  return serialized;
}

std::vector<std::uint8_t> serializedMessageBytes(const rclcpp::SerializedMessage & serialized)
{
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

RosPublishRequest makeRequest(
  const std::string & ros_topic, const std::string & interface_type, rclcpp::SerializedMessage message)
{
  RosPublishRequest request;
  request.ros_topic = ros_topic;
  request.interface_type = interface_type;
  request.message = std::move(message);
  return request;
}

template <typename MessageT>
RosPublishRequest makeRequest(const std::string & ros_topic, const MessageT & message)
{
  return makeRequest(ros_topic, rosidl_generator_traits::name<MessageT>(), serializeMessage(message));
}

std::vector<std::uint8_t> payloadBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

std::vector<std::uint8_t> makePayload(
  const std::string & ros_topic, const std::string & interface_type, const rclcpp::SerializedMessage & message)
{
  const std::string payload =
    nlohmann::json{
      {"topic", ros_topic},
      {"interface_type", interface_type},
      {"message", protocol::cdr::serialize(serializedMessageBytes(message))},
    }
      .dump();
  return payloadBytes(payload);
}

template <typename MessageT>
std::vector<std::uint8_t> makePayload(const std::string & ros_topic, const MessageT & message)
{
  return makePayload(ros_topic, rosidl_generator_traits::name<MessageT>(), serializeMessage(message));
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

  void resetPublisher(const std::string & node_prefix, const std::string & node_namespace = "/")
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
    publisher_node_ = std::make_shared<rclcpp::Node>(nextNodeName(node_prefix), node_namespace, publisher_options);

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

  RosTopicPublisher makePublisher(AccessPolicy access_policy) const
  {
    return RosTopicPublisher(
      publisher_node_->get_node_topics_interface(),
      publisher_node_->get_node_graph_interface(),
      publisher_node_->get_clock(),
      std::move(access_policy));
  }

  RosTopicPublisher makePublisher(AccessPolicy access_policy, std::size_t max_topics) const
  {
    return RosTopicPublisher(
      publisher_node_->get_node_topics_interface(),
      publisher_node_->get_node_graph_interface(),
      publisher_node_->get_clock(),
      std::move(access_policy),
      max_topics);
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

  std::size_t countPublishers(const std::string & topic)
  {
    spinSome();
    return observer_node_->count_publishers(topic);
  }

  bool waitForPublisherSubscriberMatch(
    const std::string & topic, std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    return spinUntil(
      [&]() {
        return publisher_node_->count_subscribers(topic) != 0U && observer_node_->count_publishers(topic) != 0U;
      },
      timeout);
  }

  bool waitForTopicType(
    const std::string & topic,
    const std::string & interface_type,
    std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    return spinUntil(
      [&]() {
        const auto topics = publisher_node_->get_node_graph_interface()->get_topic_names_and_types();
        const auto found = topics.find(topic);
        return found != topics.end() && found->second.size() == 1U && found->second.front() == interface_type;
      },
      timeout);
  }

  template <typename MessageT>
  bool waitForTopicType(const std::string & topic, std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    return waitForTopicType(topic, rosidl_generator_traits::name<MessageT>(), timeout);
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

TEST(TopicPublisherTest, PublishesMessagesToRequestedTopic)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/cmd";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(topic));
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  publisher.publish("alice", makeRequest(topic, message));
  ASSERT_TRUE(harness.waitForPublisherSubscriberMatch(topic));
  received_message.reset();
  publisher.publish("alice", makeRequest(topic, message));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
}

TEST(TopicPublisherTest, PayloadParsesAndPublishesMessageToRequestedTopic)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/payload_cmd";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(topic));
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.75F;

  const std::vector<std::uint8_t> payload = makePayload(topic, message);
  publisher.handlePayload("alice", payload);
  ASSERT_TRUE(harness.waitForPublisherSubscriberMatch(topic));
  received_message.reset();
  publisher.handlePayload("alice", payload);

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 48.5F, 1e-6F);
  EXPECT_NEAR(received_message->percentage, 0.75F, 1e-6F);
}

TEST(TopicPublisherTest, PayloadResolvesTopicWithPublisherNodeContext)
{
  RosTopicPublisherHarness harness;
  harness.resetPublisher("topic_publisher_node", "/robot");
  const std::string request_topic = "battery/namespaced_cmd";
  const std::string resolved_topic =
    harness.publisherNode().get_node_topics_interface()->resolve_topic_name(request_topic);

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    resolved_topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({resolved_topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(resolved_topic));
  sensor_msgs::msg::BatteryState message;
  message.voltage = 24.5F;

  const std::vector<std::uint8_t> payload = makePayload(request_topic, message);
  publisher.handlePayload("alice", payload);
  ASSERT_TRUE(harness.waitForPublisherSubscriberMatch(resolved_topic));
  received_message.reset();
  publisher.handlePayload("alice", payload);

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, 24.5F, 1e-6F);
}

TEST(TopicPublisherTest, InvalidPayloadIsDroppedWithoutPublishing)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/invalid_payload";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));

  EXPECT_NO_THROW(publisher.handlePayload("alice", payloadBytes("{")));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, PayloadWithoutRequesterIdentityIsDropped)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/anonymous_payload";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  sensor_msgs::msg::BatteryState message;

  publisher.handlePayload("", makePayload(topic, message));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, RejectsDeniedPublishRequests)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/blocked";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({"/battery/allowed"}));

  sensor_msgs::msg::BatteryState message;

  publisher.publish("alice", makeRequest(topic, message));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, CacheSizeOneRejectsNewTopicAndKeepsExistingPublisher)
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

  auto publisher = harness.makePublisher(makeAccessPolicy({"/battery/*"}), 1U);
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(first_topic));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(second_topic));

  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  publisher.publish("alice", makeRequest(first_topic, message));
  ASSERT_TRUE(harness.waitForPublisherSubscriberMatch(first_topic));
  const auto first_topic_deliveries_before_confirmed_publish = first_topic_voltages.size();
  publisher.publish("alice", makeRequest(first_topic, message));

  ASSERT_TRUE(
    harness.spinUntil([&]() { return first_topic_voltages.size() > first_topic_deliveries_before_confirmed_publish; }));
  EXPECT_NEAR(first_topic_voltages.back(), 48.5F, 1e-6F);
  EXPECT_EQ(harness.countPublishers(first_topic), 1U);

  message.voltage = 47.0F;
  publisher.publish("alice", makeRequest(second_topic, message));

  EXPECT_FALSE(harness.spinUntil([&]() { return second_topic_voltages.size() == 1U; }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.countPublishers(first_topic), 1U);
  EXPECT_EQ(harness.countPublishers(second_topic), 0U);

  const auto first_topic_deliveries_before_final_publish = first_topic_voltages.size();
  message.voltage = 50.0F;
  publisher.publish("alice", makeRequest(first_topic, message));

  ASSERT_TRUE(
    harness.spinUntil([&]() { return first_topic_voltages.size() > first_topic_deliveries_before_final_publish; }));
  EXPECT_NEAR(first_topic_voltages.back(), 50.0F, 1e-6F);
  EXPECT_EQ(harness.countPublishers(first_topic), 1U);
  EXPECT_EQ(harness.countPublishers(second_topic), 0U);
}

TEST(TopicPublisherTest, CachedPublisherPinsTypeOnCacheHits)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/cached_type";

  std::vector<float> received_voltages;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_voltages](const sensor_msgs::msg::BatteryState & message) {
      received_voltages.push_back(message.voltage);
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(topic));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  publisher.publish("alice", makeRequest(topic, first_message));
  ASSERT_TRUE(harness.waitForPublisherSubscriberMatch(topic));
  const auto deliveries_before_second_publish = received_voltages.size();

  sensor_msgs::msg::BatteryState second_message;
  second_message.voltage = 49.0F;
  publisher.publish("alice", makeRequest(topic, second_message));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_voltages.size() > deliveries_before_second_publish; }));
  EXPECT_NEAR(received_voltages.back(), 49.0F, 1e-6F);

  const auto deliveries_after_second_publish = received_voltages.size();
  std_msgs::msg::String wrong_message;
  wrong_message.data = "wrong type";
  publisher.publish("alice", makeRequest(topic, wrong_message));

  EXPECT_FALSE(harness.spinUntil(
    [&]() { return received_voltages.size() > deliveries_after_second_publish; }, std::chrono::milliseconds(200)));
  EXPECT_EQ(harness.countPublishers(topic), 1U);
}

TEST(TopicPublisherTest, RejectsRequestsWhoseDeclaredTypeDoesNotMatchTheGraph)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/invalid";

  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] const auto subscription = harness.observerNode().create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(topic));

  std_msgs::msg::String wrong_message;

  publisher.publish("alice", makeRequest(topic, wrong_message));

  expectTopicNotPublished(harness, topic, received_message);
}

TEST(TopicPublisherTest, RejectsRequestsForTopicsMissingFromTheGraph)
{
  RosTopicPublisherHarness harness;
  const std::string topic = "/battery/missing";

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));

  sensor_msgs::msg::BatteryState message;

  publisher.publish("alice", makeRequest(topic, message));

  EXPECT_FALSE(harness.spinUntil(
    [&]() { return harness.observerNode().count_publishers(topic) != 0U; }, std::chrono::milliseconds(200)));
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

  auto publisher = harness.makePublisher(makeAccessPolicy({topic}));
  ASSERT_TRUE(harness.waitForTopicType<sensor_msgs::msg::BatteryState>(topic));

  sensor_msgs::msg::BatteryState first_message;
  first_message.voltage = 48.5F;
  publisher.publish("alice", makeRequest(topic, first_message));

  ASSERT_TRUE(harness.spinUntil(
    [&]() { return harness.publisherNode().count_subscribers(topic) == 1U; }, std::chrono::seconds(5)));
  received_message.reset();
  publisher.publish("alice", makeRequest(topic, first_message));

  ASSERT_TRUE(harness.spinUntil([&]() { return received_message.has_value(); }));
  EXPECT_EQ(harness.countPublishers(topic), 1U);

  publisher.shutdown();
  publisher.shutdown();

  ASSERT_TRUE(harness.spinUntil([&]() { return harness.countPublishers(topic) == 0U; }));

  received_message.reset();
  sensor_msgs::msg::BatteryState late_message;

  publisher.publish("alice", makeRequest(topic, late_message));

  EXPECT_FALSE(harness.spinUntil([&]() { return received_message.has_value(); }, std::chrono::milliseconds(200)));
}

}  // namespace
}  // namespace livekit_ros2_bridge
