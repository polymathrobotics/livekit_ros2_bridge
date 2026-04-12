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
#include <stdexcept>
#include <string>
#include <vector>

#include "data_track_publisher.hpp"
#include "fake_room_session.hpp"
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

TEST(DataTrackPublisherTest, PublishTrackReportsFailureAndDoesNotRetainTrackOnPublishError)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("test_publish_track_failure_node");
  const std::string topic = "/battery/publish_failure";
  auto ros_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)ros_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  std::vector<std::string> publish_requests;
  SubscriptionRegistry registry(
    *node,
    [](const std::string &, const std::uint8_t *, std::size_t) {},
    [&publish_requests](const std::string & name, std::size_t) { publish_requests.push_back(name); },
    [](const std::string &) {},
    nullptr);

  FakeRoomSession session;
  session.state->publish_data_track_handler = [](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    throw std::runtime_error("simulated publish failure");
  };
  DataTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_EQ(publish_requests.size(), 1U);
  EXPECT_EQ(response.track_name, publish_requests[0]);

  EXPECT_NO_THROW(publisher.publishTrack(publish_requests[0], 0, registry));
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], publish_requests[0]);
  EXPECT_TRUE(session.state->unpublish_attempted_data_track_names.empty());

  const auto retry_response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_EQ(publish_requests.size(), 2U);
  EXPECT_EQ(retry_response.track_name, publish_requests[0]);
  EXPECT_EQ(publish_requests[1], publish_requests[0]);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_TRUE(session.state->unpublish_attempted_data_track_names.empty());
}

TEST(DataTrackPublisherTest, PublishTrackImmediatelyReclaimsStaleSubscriptionTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("test_stale_track_node");
  const std::string topic = "/battery/stale";
  auto ros_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)ros_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(
    *node,
    [](const std::string &, const std::uint8_t *, std::size_t) {},
    [](const std::string &, std::size_t) {},
    [](const std::string &) {},
    nullptr);

  FakeRoomSession session;
  DataTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_TRUE(registry.hasSubscription(topic));

  registry.resetSessionState();
  ASSERT_FALSE(registry.hasSubscription(topic));

  publisher.publishTrack(response.track_name, 0, registry);

  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], response.track_name);
  ASSERT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names[0], response.track_name);
  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], response.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
}

TEST(DataTrackPublisherTest, UnpublishTrackSwallowsSessionErrorAndRemovesTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("test_unpublish_track_node");
  const std::string topic = "/battery/unpublish";
  auto ros_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)ros_publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(
    *node,
    [](const std::string &, const std::uint8_t *, std::size_t) {},
    [](const std::string &, std::size_t) {},
    [](const std::string &) {},
    nullptr);

  FakeRoomSession session;
  DataTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  publisher.publishTrack(response.track_name, 0, registry);

  session.state->unpublish_rejected_data_track_names.push_back(response.track_name);

  EXPECT_NO_THROW(publisher.unpublishTrack(response.track_name));
  ASSERT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names[0], response.track_name);
  EXPECT_TRUE(session.state->unpublished_data_track_names.empty());

  EXPECT_NO_THROW(publisher.unpublishTrack(response.track_name));
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
}

TEST(DataTrackPublisherTest, UnpublishAllUnpublishesAcceptedTracksAndContinuesOnError)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("test_clear_tracks_node");
  const std::string topic_a = "/battery/clear_a";
  const std::string topic_b = "/battery/clear_b";
  auto publisher_a = node->create_publisher<sensor_msgs::msg::BatteryState>(topic_a, rclcpp::QoS(10));
  auto publisher_b = node->create_publisher<sensor_msgs::msg::BatteryState>(topic_b, rclcpp::QoS(10));
  (void)publisher_a;
  (void)publisher_b;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic_a, "sensor_msgs/msg/BatteryState"));
  ASSERT_TRUE(waitForTopicType(executor, node, topic_b, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(
    *node,
    [](const std::string &, const std::uint8_t *, std::size_t) {},
    [](const std::string &, std::size_t) {},
    [](const std::string &) {},
    nullptr);

  FakeRoomSession session;
  DataTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response_a = registry.renewSubscription("alice", topic_a, 0, expiry);
  const auto response_b = registry.renewSubscription("alice", topic_b, 0, expiry);
  publisher.publishTrack(response_a.track_name, 0, registry);
  publisher.publishTrack(response_b.track_name, 0, registry);

  session.state->unpublish_rejected_data_track_names.push_back(response_b.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());

  auto attempts = session.state->unpublish_attempted_data_track_names;
  std::sort(attempts.begin(), attempts.end());
  ASSERT_EQ(attempts.size(), 2U);
  EXPECT_EQ(attempts[0], response_a.track_name);
  EXPECT_EQ(attempts[1], response_b.track_name);

  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], response_a.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names.size(), 2U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
