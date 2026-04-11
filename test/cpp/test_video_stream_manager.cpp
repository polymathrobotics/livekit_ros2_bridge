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
#include "sensor_msgs/msg/compressed_image.hpp"
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

sensor_msgs::msg::Image makeRgbImage(std::uint32_t width = 2, std::uint32_t height = 2)
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.header.stamp.nanosec = 2000;
  image.width = width;
  image.height = height;
  image.encoding = "rgb8";
  image.step = width * 3U;
  image.data.resize(static_cast<std::size_t>(image.step) * height);
  for (std::size_t idx = 0; idx < image.data.size(); idx += 3U) {
    image.data[idx] = 255;
    if (idx + 1U < image.data.size()) {
      image.data[idx + 1U] = 255;
    }
    if (idx + 2U < image.data.size()) {
      image.data[idx + 2U] = 255;
    }
  }
  return image;
}

sensor_msgs::msg::CompressedImage makeTransportStyleJpegImage()
{
  sensor_msgs::msg::CompressedImage image;
  image.header.stamp.sec = 1;
  image.header.stamp.nanosec = 2000;
  image.format = "rgb8; jpeg compressed bgr8";
  image.data = {
    255, 216, 255, 224, 0,   16,  74,  70,  73,  70,  0,   1,   1,   0,   0,   1,   0,   1,   0,   0,   255, 219, 0,
    67,  0,   8,   6,   6,   7,   6,   5,   8,   7,   7,   7,   9,   9,   8,   10,  12,  20,  13,  12,  11,  11,  12,
    25,  18,  19,  15,  20,  29,  26,  31,  30,  29,  26,  28,  28,  32,  36,  46,  39,  32,  34,  44,  35,  28,  28,
    40,  55,  41,  44,  48,  49,  52,  52,  52,  31,  39,  57,  61,  56,  50,  60,  46,  51,  52,  50,  255, 219, 0,
    67,  1,   9,   9,   9,   12,  11,  12,  24,  13,  13,  24,  50,  33,  28,  33,  50,  50,  50,  50,  50,  50,  50,
    50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,
    50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  255, 192,
    0,   17,  8,   0,   2,   0,   2,   3,   1,   34,  0,   2,   17,  1,   3,   17,  1,   255, 196, 0,   31,  0,   0,
    1,   5,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   1,   2,   3,   4,   5,   6,   7,
    8,   9,   10,  11,  255, 196, 0,   181, 16,  0,   2,   1,   3,   3,   2,   4,   3,   5,   5,   4,   4,   0,   0,
    1,   125, 1,   2,   3,   0,   4,   17,  5,   18,  33,  49,  65,  6,   19,  81,  97,  7,   34,  113, 20,  50,  129,
    145, 161, 8,   35,  66,  177, 193, 21,  82,  209, 240, 36,  51,  98,  114, 130, 9,   10,  22,  23,  24,  25,  26,
    37,  38,  39,  40,  41,  42,  52,  53,  54,  55,  56,  57,  58,  67,  68,  69,  70,  71,  72,  73,  74,  83,  84,
    85,  86,  87,  88,  89,  90,  99,  100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120, 121, 122, 131,
    132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162, 163, 164, 165, 166, 167, 168,
    169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194, 195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212,
    213, 214, 215, 216, 217, 218, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 241, 242, 243, 244, 245, 246, 247,
    248, 249, 250, 255, 196, 0,   31,  1,   0,   3,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,
    0,   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  255, 196, 0,   181, 17,  0,   2,   1,   2,   4,
    4,   3,   4,   7,   5,   4,   4,   0,   1,   2,   119, 0,   1,   2,   3,   17,  4,   5,   33,  49,  6,   18,  65,
    81,  7,   97,  113, 19,  34,  50,  129, 8,   20,  66,  145, 161, 177, 193, 9,   35,  51,  82,  240, 21,  98,  114,
    209, 10,  22,  36,  52,  225, 37,  241, 23,  24,  25,  26,  38,  39,  40,  41,  42,  53,  54,  55,  56,  57,  58,
    67,  68,  69,  70,  71,  72,  73,  74,  83,  84,  85,  86,  87,  88,  89,  90,  99,  100, 101, 102, 103, 104, 105,
    106, 115, 116, 117, 118, 119, 120, 121, 122, 130, 131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150,
    151, 152, 153, 154, 162, 163, 164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194,
    195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 226, 227, 228, 229, 230, 231,
    232, 233, 234, 242, 243, 244, 245, 246, 247, 248, 249, 250, 255, 218, 0,   12,  3,   1,   0,   2,   17,  3,   17,
    0,   63,  0,   226, 232, 162, 138, 249, 147, 247, 19,  255, 217,
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

VideoStreamSpec makeRosSpec(const std::string & topic, const std::string & track_name)
{
  VideoStreamSpec spec;
  spec.stream_key = "topic:" + topic;
  spec.track_name = track_name;
  spec.ros_topic = topic;
  spec.interface_type = kImageInterfaceType;
  spec.source_kind = VideoSourceKind::RosTopic;
  spec.ingest_mode = kRawImageIngestMode;
  spec.selected_config_key = "default_ros";
  spec.transform_description = "queue max-size-buffers=2 leaky=downstream";
  return spec;
}

VideoStreamSpec makeCompressedRosSpec(const std::string & topic, const std::string & track_name)
{
  auto spec = makeRosSpec(topic, track_name);
  spec.interface_type = kCompressedImageInterfaceType;
  spec.ingest_mode = kCompressedImageIngestMode;
  return spec;
}

VideoStreamSpec makeExternalSpec(const std::string & external_name, const std::string & track_name)
{
  VideoStreamSpec spec;
  spec.stream_key = "external:" + external_name;
  spec.track_name = track_name;
  spec.external_name = external_name;
  spec.source_kind = VideoSourceKind::External;
  spec.ingest_mode = kExternalIngestMode;
  spec.selected_config_key = external_name;
  spec.source_description = "videotestsrc is-live=true pattern=black";
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

TEST_F(VideoStreamManagerTest, ReliableRosPublisherPublishesTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_reliable_ros"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const std::string topic = "/camera/reliable";
  const auto spec = makeRosSpec(topic, "ros.video.camera.reliable");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(
    executor, publisher, image, [&session]() { return session.state->published_video_track_names.size() == 1U; }));

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], spec.track_name);
}

TEST_F(VideoStreamManagerTest, CompressedRosPublisherAcceptsImageTransportStyleJpegFormat)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_compressed_ros"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const std::string topic = "/camera/compressed";
  const auto spec = makeCompressedRosSpec(topic, "ros.video.camera.compressed");
  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(manager.ensureStream(spec), spec.track_name);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeTransportStyleJpegImage();
  ASSERT_TRUE(publishUntil(
    executor,
    publisher,
    image,
    [&session]() { return session.state->published_video_track_names.size() == 1U; },
    std::chrono::seconds(5)));

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], spec.track_name);
}

TEST_F(VideoStreamManagerTest, ExternalPipelinePublishesTrackAndStopUnpublishesIt)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_external"));
  FakeRoomSession session;
  VideoStreamManager manager(*node, session);

  const auto spec = makeExternalSpec("/sources/front", "ros.video.external.sources.front");

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

  const auto spec = makeExternalSpec("/sources/shutdown", "ros.video.external.sources.shutdown");
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

TEST_F(VideoStreamManagerTest, GlobalPublishConfigIsAppliedToEveryPublishedTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_manager_publish_config"));
  FakeRoomSession session;

  VideoPublishConfig publish_config;
  publish_config.codec = VideoPublishCodec::H264;
  publish_config.max_bitrate_bps = 900000;
  publish_config.max_framerate = 24.0;
  publish_config.simulcast = VideoPublishSimulcast::Enabled;
  VideoStreamManager manager(*node, session, nullptr, publish_config);

  const std::string first_topic = "/camera/publish_config/one";
  const std::string second_topic = "/camera/publish_config/two";
  const auto first_spec = makeRosSpec(first_topic, "ros.video.camera.publish_config.one");
  const auto second_spec = makeRosSpec(second_topic, "ros.video.camera.publish_config.two");
  auto first_publisher = node->create_publisher<sensor_msgs::msg::Image>(first_topic, rclcpp::QoS(10));
  auto second_publisher = node->create_publisher<sensor_msgs::msg::Image>(second_topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(manager.ensureStream(first_spec), first_spec.track_name);
  EXPECT_EQ(manager.ensureStream(second_spec), second_spec.track_name);
  ASSERT_TRUE(spinUntil(executor, [&first_publisher]() { return first_publisher->get_subscription_count() == 1U; }));
  ASSERT_TRUE(spinUntil(executor, [&second_publisher]() { return second_publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(executor, first_publisher, makeRgbImage(2, 2), [&session]() {
    return session.state->published_video_configs.size() == 1U;
  }));
  ASSERT_TRUE(publishUntil(executor, second_publisher, makeRgbImage(4, 4), [&session]() {
    return session.state->published_video_configs.size() == 2U;
  }));

  ASSERT_EQ(session.state->published_video_track_names.size(), 2U);
  EXPECT_EQ(session.state->published_video_track_names[0], first_spec.track_name);
  EXPECT_EQ(session.state->published_video_track_names[1], second_spec.track_name);
  ASSERT_EQ(session.state->published_video_configs.size(), 2U);
  for (const auto & config : session.state->published_video_configs) {
    EXPECT_EQ(config.codec, publish_config.codec);
    EXPECT_EQ(config.max_bitrate_bps, publish_config.max_bitrate_bps);
    EXPECT_DOUBLE_EQ(config.max_framerate, publish_config.max_framerate);
    EXPECT_EQ(config.simulcast, publish_config.simulcast);
  }
}

}  // namespace livekit_ros2_bridge
