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
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video_stream_manager.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitUntil;

std::string nextNodeName(const std::string & prefix)
{
  static std::atomic<size_t> next_suffix{0};
  return prefix + "_" + std::to_string(next_suffix.fetch_add(1));
}

sensor_msgs::msg::Image makeRgbImage()
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.header.stamp.nanosec = 2000;
  image.width = 2;
  image.height = 2;
  image.encoding = "rgb8";
  image.step = 6;
  image.data = {
    255,
    0,
    0,
    0,
    255,
    0,
    0,
    0,
    255,
    255,
    255,
    255,
  };
  return image;
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

SidecarLaunchSpec makeRosSpec(const std::string & topic, const std::string & track_name)
{
  SidecarLaunchSpec spec;
  spec.stream_key = "topic:" + topic;
  spec.track_name = track_name;
  spec.ros_topic = topic;
  spec.interface_type = kImageInterfaceType;
  spec.source_kind = VideoSourceKind::RosTopic;
  spec.ingest_mode = kRawImageIngestMode;
  spec.selected_config_key = "default_ros";
  spec.pipeline_description = "queue max-size-buffers=2 leaky=downstream";
  return spec;
}

SidecarLaunchSpec makePipelineSpec(const std::string & external_name, const std::string & track_name)
{
  SidecarLaunchSpec spec;
  spec.stream_key = "external:" + external_name;
  spec.track_name = track_name;
  spec.external_name = external_name;
  spec.source_kind = VideoSourceKind::Pipeline;
  spec.ingest_mode = kPipelineIngestMode;
  spec.selected_config_key = external_name;
  spec.pipeline_description = "videotestsrc is-live=true pattern=black";
  return spec;
}

}  // namespace

class VideoStreamManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(VideoStreamManagerTest, SharedRosStreamUsesSingleSubscriptionAndSinglePublishedTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_shared_ros"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const std::string topic = "/camera/shared";
  const auto spec = makeRosSpec(topic, "ros.video.camera.shared");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);
  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(
    executor, publisher, image, [&session]() { return session.state->published_video_track_names.size() == 1U; }));

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], spec.track_name);
}

TEST_F(VideoStreamManagerTest, StopStreamUnpublishesRosTrackAndRemovesSubscription)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_stop_ros"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const std::string topic = "/camera/stop";
  const auto spec = makeRosSpec(topic, "ros.video.camera.stop");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  manager.ensureStream(spec);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(
    executor, publisher, image, [&session]() { return session.state->published_video_track_names.size() == 1U; }));

  manager.stopStream(spec.stream_key);

  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], spec.track_name);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 0U; }));
}

TEST_F(VideoStreamManagerTest, ExternalPipelinePublishesTrackAndStopUnpublishesIt)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_external"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const auto spec = makePipelineSpec("/sources/front", "ros.video.external.sources.front");

  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);

  ASSERT_TRUE(waitUntil([&session]() { return session.state->published_video_track_names.size() == 1U; }));
  EXPECT_EQ(session.state->published_video_track_names[0], spec.track_name);

  manager.stopStream(spec.stream_key);

  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], spec.track_name);
}

TEST_F(VideoStreamManagerTest, ShutdownUnpublishesActiveTracksAndRejectsNewStreams)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_shutdown"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const auto spec = makePipelineSpec("/sources/shutdown", "ros.video.external.sources.shutdown");
  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);

  ASSERT_TRUE(waitUntil([&session]() { return session.state->published_video_track_names.size() == 1U; }));

  manager.shutdown();

  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], spec.track_name);

  try {
    (void)manager.ensureStream(spec);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), "Video stream manager is shut down.");
  }
}

}  // namespace livekit_ros2_bridge
