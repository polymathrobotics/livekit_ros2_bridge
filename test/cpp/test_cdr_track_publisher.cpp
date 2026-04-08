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
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cdr_track_publisher.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "room_session.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "subscription_registry.hpp"

namespace livekit_ros2_bridge
{
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

class RecordingRoomSession final : public RoomSession
{
public:
  void start(
    RoomConnectionConfig,
    std::shared_ptr<AccessTokenSource>,
    RoomSessionCallbacks,
    std::chrono::milliseconds,
    std::chrono::milliseconds,
    std::chrono::seconds) override
  {}

  void stop() override
  {}

  bool registerRpcMethod(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpcMethod(const std::string &) override
  {
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket &) override
  {}

  std::shared_ptr<livekit::LocalDataTrack> publishCdrTrack(const std::string & name) override
  {
    publish_attempts.push_back(name);
    if (throw_on_publish) {
      throw std::runtime_error("simulated publish failure");
    }

    auto owner = std::make_shared<int>(next_track_id_++);
    auto track =
      std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
    track_names_[track.get()] = name;
    return track;
  }

  void unpublishCdrTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    const auto it = track_names_.find(track.get());
    const std::string name = (it == track_names_.end()) ? "<unknown>" : it->second;
    unpublish_attempts.push_back(name);

    if (throw_on_unpublish_names.count(name) > 0U) {
      throw std::runtime_error("simulated unpublish failure");
    }

    unpublished_track_names.push_back(name);
  }

  bool isVideoPublisherHealthy(const std::string &) const override
  {
    return false;
  }

  bool throw_on_publish = false;
  std::vector<std::string> publish_attempts;
  std::vector<std::string> unpublish_attempts;
  std::vector<std::string> unpublished_track_names;
  std::unordered_set<std::string> throw_on_unpublish_names;

private:
  int next_track_id_ = 1;
  std::unordered_map<const livekit::LocalDataTrack *, std::string> track_names_;
};

TEST(CdrTrackPublisherTest, PublishTrackReportsFailureAndDoesNotRetainTrackOnPublishError)
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

  RecordingRoomSession session;
  session.throw_on_publish = true;
  CdrTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_EQ(publish_requests.size(), 1U);
  EXPECT_EQ(response.track_name, publish_requests[0]);

  EXPECT_NO_THROW(publisher.publishTrack(publish_requests[0], 0, registry));
  ASSERT_EQ(session.publish_attempts.size(), 1U);
  EXPECT_EQ(session.publish_attempts[0], publish_requests[0]);
  EXPECT_TRUE(session.unpublish_attempts.empty());

  const auto retry_response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_EQ(publish_requests.size(), 2U);
  EXPECT_EQ(retry_response.track_name, publish_requests[0]);
  EXPECT_EQ(publish_requests[1], publish_requests[0]);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_TRUE(session.unpublish_attempts.empty());
}

TEST(CdrTrackPublisherTest, PublishTrackImmediatelyReclaimsStaleSubscriptionTrack)
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

  RecordingRoomSession session;
  CdrTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  ASSERT_TRUE(registry.hasSubscription(topic));

  registry.resetSessionState();
  ASSERT_FALSE(registry.hasSubscription(topic));

  publisher.publishTrack(response.track_name, 0, registry);

  ASSERT_EQ(session.publish_attempts.size(), 1U);
  EXPECT_EQ(session.publish_attempts[0], response.track_name);
  ASSERT_EQ(session.unpublish_attempts.size(), 1U);
  EXPECT_EQ(session.unpublish_attempts[0], response.track_name);
  ASSERT_EQ(session.unpublished_track_names.size(), 1U);
  EXPECT_EQ(session.unpublished_track_names[0], response.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_EQ(session.unpublish_attempts.size(), 1U);
}

TEST(CdrTrackPublisherTest, UnpublishTrackSwallowsSessionErrorAndRemovesTrack)
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

  RecordingRoomSession session;
  CdrTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response = registry.renewSubscription("alice", topic, 0, expiry);
  publisher.publishTrack(response.track_name, 0, registry);

  session.throw_on_unpublish_names.insert(response.track_name);

  EXPECT_NO_THROW(publisher.unpublishTrack(response.track_name));
  ASSERT_EQ(session.unpublish_attempts.size(), 1U);
  EXPECT_EQ(session.unpublish_attempts[0], response.track_name);
  EXPECT_TRUE(session.unpublished_track_names.empty());

  EXPECT_NO_THROW(publisher.unpublishTrack(response.track_name));
  EXPECT_EQ(session.unpublish_attempts.size(), 1U);
}

TEST(CdrTrackPublisherTest, ClearUnpublishesAcceptedTracksAndContinuesOnError)
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

  RecordingRoomSession session;
  CdrTrackPublisher publisher(session, node->get_clock());

  const auto expiry = std::chrono::steady_clock::now() + std::chrono::hours(1);
  const auto response_a = registry.renewSubscription("alice", topic_a, 0, expiry);
  const auto response_b = registry.renewSubscription("alice", topic_b, 0, expiry);
  publisher.publishTrack(response_a.track_name, 0, registry);
  publisher.publishTrack(response_b.track_name, 0, registry);

  session.throw_on_unpublish_names.insert(response_b.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());

  auto attempts = session.unpublish_attempts;
  std::sort(attempts.begin(), attempts.end());
  ASSERT_EQ(attempts.size(), 2U);
  EXPECT_EQ(attempts[0], response_a.track_name);
  EXPECT_EQ(attempts[1], response_b.track_name);

  ASSERT_EQ(session.unpublished_track_names.size(), 1U);
  EXPECT_EQ(session.unpublished_track_names[0], response_a.track_name);

  EXPECT_NO_THROW(publisher.unpublishAll());
  EXPECT_EQ(session.unpublish_attempts.size(), 2U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
