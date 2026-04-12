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

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "data_track_publisher.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"

namespace livekit_ros2_bridge
{
namespace
{

using test_support::ScopedRclcppInit;

TEST(DataTrackPublisherTest, PublishFailureInvokesFailureCallbackWithoutRetainingTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_publish_failure_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.publish_failure";
  session.state->publish_data_track_handler = [](const std::string &) -> std::shared_ptr<livekit::LocalDataTrack> {
    throw std::runtime_error("simulated publish failure");
  };

  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  EXPECT_NO_THROW(
    publisher.publish(7, [](std::size_t) { return true; }, [&publish_failed]() { publish_failed = true; }));
  EXPECT_TRUE(publish_failed);

  publisher.shutdown();
  EXPECT_TRUE(session.state->unpublish_attempted_data_track_names.empty());
}

TEST(DataTrackPublisherTest, RejectedPublishIsImmediatelyReclaimedAndNotRetained)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_stale_reclaim_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.stale_reclaim";
  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  publisher.publish(11, [](std::size_t) { return false; }, [&publish_failed]() { publish_failed = true; });
  EXPECT_FALSE(publish_failed);
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  publisher.shutdown();
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

TEST(DataTrackPublisherTest, RejectedRepublishDoesNotDisplaceExistingActiveTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_rejected_republish_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.rejected_republish";
  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  publisher.publish(13, [](std::size_t) { return true; }, [&publish_failed]() { publish_failed = true; });
  ASSERT_FALSE(publish_failed);

  const std::array<std::uint8_t, 2> payload_before_reject{0x01U, 0x02U};
  const std::array<std::uint8_t, 3> payload_after_reject{0x03U, 0x04U, 0x05U};
  publisher.tryPush(payload_before_reject.data(), payload_before_reject.size());

  publisher.publish(17, [](std::size_t) { return false; }, [&publish_failed]() { publish_failed = true; });
  publisher.tryPush(payload_after_reject.data(), payload_after_reject.size());
  publisher.shutdown();

  EXPECT_FALSE(publish_failed);
  ASSERT_EQ(session.state->pushed_data_track_frames.size(), 2U);
  EXPECT_EQ(
    session.state->pushed_data_track_frames[0].payload,
    std::vector<std::uint8_t>(payload_before_reject.begin(), payload_before_reject.end()));
  EXPECT_EQ(
    session.state->pushed_data_track_frames[1].payload,
    std::vector<std::uint8_t>(payload_after_reject.begin(), payload_after_reject.end()));
  EXPECT_EQ(session.state->unpublished_data_track_names, (std::vector<std::string>{track_name, track_name}));
}

TEST(DataTrackPublisherTest, PublishAcceptedCallbackExceptionReportsFailureAndReclaimsTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_publish_callback_failure_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.publish_callback_failure";
  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  EXPECT_NO_THROW(publisher.publish(
    29,
    [](std::size_t) -> bool { throw std::runtime_error("simulated publish acceptance failure"); },
    [&publish_failed]() { publish_failed = true; }));

  EXPECT_TRUE(publish_failed);
  EXPECT_EQ(session.state->published_data_track_names, std::vector<std::string>{track_name});
  EXPECT_EQ(session.state->unpublished_data_track_names, std::vector<std::string>{track_name});

  publisher.shutdown();
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

TEST(DataTrackPublisherTest, QueueFullPushDropsFrameAndKeepsTrackPublishedUntilShutdown)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_queue_full_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.queue_full";
  session.state->try_push_data_track_handler = [](const std::string &, const std::vector<std::uint8_t> &) {
    return DataTrackPushResult::failure(DataTrackPushError{DataTrackPushErrorCode::kQueueFull, "queue full"});
  };

  DataTrackPublisher publisher(session, track_name, node->get_clock());
  publisher.publish(5, [](std::size_t) { return true; }, []() {});

  const std::array<std::uint8_t, 3> payload{0x01U, 0x02U, 0x03U};
  publisher.tryPush(payload.data(), payload.size());

  EXPECT_TRUE(session.state->pushed_data_track_frames.empty());
  EXPECT_TRUE(session.state->unpublish_attempted_data_track_names.empty());

  publisher.shutdown();

  EXPECT_EQ(session.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

TEST(DataTrackPublisherTest, UnpublishClearsActiveTrackAndMakesShutdownIdempotent)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_unpublish_lifecycle_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.unpublish_lifecycle";
  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  publisher.publish(19, [](std::size_t) { return true; }, [&publish_failed]() { publish_failed = true; });
  ASSERT_FALSE(publish_failed);

  const std::array<std::uint8_t, 3> payload_before_unpublish{0x01U, 0x02U, 0x03U};
  const std::array<std::uint8_t, 2> payload_after_unpublish{0x09U, 0x0AU};
  publisher.tryPush(payload_before_unpublish.data(), payload_before_unpublish.size());
  publisher.unpublish();
  publisher.tryPush(payload_after_unpublish.data(), payload_after_unpublish.size());
  publisher.shutdown();
  publisher.shutdown();

  ASSERT_EQ(session.state->pushed_data_track_frames.size(), 1U);
  EXPECT_EQ(
    session.state->pushed_data_track_frames.front().payload,
    std::vector<std::uint8_t>(payload_before_unpublish.begin(), payload_before_unpublish.end()));
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

TEST(DataTrackPublisherTest, ShutdownAfterUnpublishFailureDoesNotRetryOrRetainTrack)
{
  ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>("data_track_publisher_unpublish_failure_test");
  FakeRoomConnection session;
  const std::string track_name = "ros.data.battery.unpublish_failure";
  session.state->unpublish_rejected_data_track_names.push_back(track_name);

  DataTrackPublisher publisher(session, track_name, node->get_clock());
  bool publish_failed = false;

  publisher.publish(23, [](std::size_t) { return true; }, [&publish_failed]() { publish_failed = true; });
  ASSERT_FALSE(publish_failed);

  const std::array<std::uint8_t, 2> payload_before_shutdown{0x05U, 0x06U};
  const std::array<std::uint8_t, 3> payload_after_failed_shutdown{0x07U, 0x08U, 0x09U};
  publisher.tryPush(payload_before_shutdown.data(), payload_before_shutdown.size());
  publisher.shutdown();
  publisher.tryPush(payload_after_failed_shutdown.data(), payload_after_failed_shutdown.size());
  publisher.shutdown();

  ASSERT_EQ(session.state->pushed_data_track_frames.size(), 1U);
  EXPECT_EQ(
    session.state->pushed_data_track_frames.front().payload,
    std::vector<std::uint8_t>(payload_before_shutdown.begin(), payload_before_shutdown.end()));
  EXPECT_EQ(session.state->unpublish_attempted_data_track_names, std::vector<std::string>{track_name});
}

}  // namespace
}  // namespace livekit_ros2_bridge
