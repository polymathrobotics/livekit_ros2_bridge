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
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "data_track_publisher.hpp"
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
  message.voltage = 48.5F;
  message.percentage = 0.75F;
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

TEST(DataTrackPublisherTest, QueueFullPushDropsFrameWithoutRecordingPayload)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_queue_full_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/queue_full";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  session.state->try_push_data_track_handler = [](const std::string &, const std::vector<std::uint8_t> &) {
    return DataTrackPushResult::failure(DataTrackPushError{DataTrackPushErrorCode::kQueueFull, "queue full"});
  };

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], response.track_name);

  ASSERT_TRUE(
    publishUntil(executor, publisher, makeBatteryState(), [&]() { return session.state->event_log.size() >= 2U; }));
  EXPECT_TRUE(session.state->pushed_data_track_frames.empty());
}

TEST(DataTrackPublisherTest, ResetSessionStateSwallowsUnpublishErrorAndClearsSubscription)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_reset_unpublish_error_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/reset_unpublish_error";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);
  session.state->unpublish_rejected_data_track_names.push_back(response.track_name);

  EXPECT_NO_THROW(registry.resetSessionState());
  EXPECT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names[0], response.track_name);
  EXPECT_TRUE(session.state->unpublished_data_track_names.empty());
}

TEST(DataTrackPublisherTest, PublishFailureInvokesFailureCallbackWithoutRetainingTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_publish_failure_test");
  FakeRoomConnection session;
  session.state->publish_data_track_handler = [](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    throw std::runtime_error("simulated publish failure");
  };

  DataTrackPublisher publisher(session, "ros.data.battery.publish_failure", node->get_clock());
  bool publish_failed = false;

  EXPECT_NO_THROW(
    publisher.publish(7, [](std::size_t) { return true; }, [&publish_failed]() { publish_failed = true; }));
  EXPECT_TRUE(publish_failed);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], "ros.data.battery.publish_failure");

  EXPECT_NO_THROW(publisher.shutdown());
  EXPECT_TRUE(session.state->unpublish_attempted_data_track_names.empty());
}

TEST(DataTrackPublisherTest, RejectedPublishIsImmediatelyReclaimedAndNotRetained)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_stale_reclaim_test");
  FakeRoomConnection session;
  DataTrackPublisher publisher(session, "ros.data.battery.stale_reclaim", node->get_clock());
  bool publish_failed = false;

  EXPECT_NO_THROW(
    publisher.publish(11, [](std::size_t) { return false; }, [&publish_failed]() { publish_failed = true; }));
  EXPECT_FALSE(publish_failed);
  ASSERT_EQ(session.state->published_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_data_track_names[0], "ros.data.battery.stale_reclaim");
  ASSERT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names[0], "ros.data.battery.stale_reclaim");
  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], "ros.data.battery.stale_reclaim");

  EXPECT_NO_THROW(publisher.shutdown());
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names.size(), 1U);
}

TEST(DataTrackPublisherTest, ShutdownUnpublishesAcceptedTrackOnce)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_shutdown_test");
  FakeRoomConnection session;
  const std::string topic = "/battery/shutdown";
  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(topic, rclcpp::QoS(10));
  (void)publisher;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(waitForTopicType(executor, node, topic, "sensor_msgs/msg/BatteryState"));

  SubscriptionRegistry registry(*node, session, nullptr);
  const auto response = registry.renewSubscription("alice", topic, 0, kFarFuture);

  EXPECT_NO_THROW(registry.shutdown());
  EXPECT_FALSE(registry.hasSubscription(topic));
  ASSERT_EQ(session.state->unpublished_data_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_data_track_names[0], response.track_name);

  EXPECT_NO_THROW(registry.shutdown());
  EXPECT_EQ(session.state->unpublished_data_track_names.size(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
