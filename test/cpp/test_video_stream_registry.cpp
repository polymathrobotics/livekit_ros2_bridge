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
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video_profiling.hpp"
#include "video_stream_registry.hpp"
#include "video_stream_spec.hpp"

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

VideoStreamConfig makeConfiguredSourceConfig(std::initializer_list<const char *> source_names)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  for (const char * source_name : source_names) {
    ConfiguredVideoStreamSource source;
    source.ingress_fragment = "videotestsrc is-live=true pattern=black";
    config.configured_sources.emplace(source_name, std::move(source));
  }
  return config;
}

VideoProfilingConfig makeProfilingConfig()
{
  VideoProfilingConfig config;
  config.enabled = true;
  config.trace_max_events = 1024;
  return config;
}

}  // namespace

class VideoStreamRegistryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(VideoStreamRegistryTest, SharedRosStreamUsesSingleSubscriptionAndSinglePublishedTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_shared_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/shared";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(executor, publisher, image, [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));

  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, ConcurrentStartSharesSingleRosSubscriptionAndTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_concurrent_shared_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/concurrent_shared";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::promise<void> first_started_promise;
  std::promise<void> second_started_promise;
  auto first_started = first_started_promise.get_future();
  auto second_started = second_started_promise.get_future();

  std::promise<void> release_promise;
  const auto release = release_promise.get_future().share();

  auto start_stream = [&registry, &topic, release](std::promise<void> started_promise) mutable {
    started_promise.set_value();
    release.wait();
    registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  };

  auto first_start = std::async(std::launch::async, start_stream, std::move(first_started_promise));
  auto second_start = std::async(std::launch::async, start_stream, std::move(second_started_promise));

  ASSERT_EQ(first_started.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  ASSERT_EQ(second_started.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  release_promise.set_value();

  ASSERT_EQ(first_start.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(second_start.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  first_start.get();
  second_start.get();

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(executor, publisher, makeRgbImage(), [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));

  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, StopUnpublishesRosTrackAndRemovesSubscription)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_stop_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/stop";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(executor, publisher, image, [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));

  registry.stop(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);

  EXPECT_EQ(room_connection.state->unpublished_video_track_names, (std::vector<std::string>{info.track_name}));
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 0U; }));
}

TEST_F(VideoStreamRegistryTest, StopBeforeFirstFrameRemovesRosSubscriptionAndAllowsRestart)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_restart_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/restart";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  registry.stop(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);

  EXPECT_TRUE(room_connection.state->unpublished_video_track_names.empty());
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 0U; }));

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(executor, publisher, makeRgbImage(), [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));
  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, StopLeavesOtherRosStreamsRunning)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_stop_one_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string first_topic = "/camera/stop_one/first";
  const std::string second_topic = "/camera/stop_one/second";
  const auto first_info = registry.resolve(SubscriptionTargetKind::Topic, first_topic, kImageInterfaceType);
  const auto second_info = registry.resolve(SubscriptionTargetKind::Topic, second_topic, kImageInterfaceType);
  auto first_publisher = node->create_publisher<sensor_msgs::msg::Image>(first_topic, rclcpp::QoS(10));
  auto second_publisher = node->create_publisher<sensor_msgs::msg::Image>(second_topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, first_topic, kImageInterfaceType);
  registry.start(SubscriptionTargetKind::Topic, second_topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&first_publisher]() { return first_publisher->get_subscription_count() == 1U; }));
  ASSERT_TRUE(spinUntil(executor, [&second_publisher]() { return second_publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(executor, first_publisher, makeRgbImage(), [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));
  ASSERT_TRUE(publishUntil(executor, second_publisher, makeRgbImage(), [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 2U;
  }));

  registry.stop(SubscriptionTargetKind::Topic, first_topic, kImageInterfaceType);

  EXPECT_EQ(room_connection.state->unpublished_video_track_names, (std::vector<std::string>{first_info.track_name}));
  ASSERT_TRUE(spinUntil(executor, [&first_publisher]() { return first_publisher->get_subscription_count() == 0U; }));
  ASSERT_TRUE(spinUntil(executor, [&second_publisher]() { return second_publisher->get_subscription_count() == 1U; }));
}

TEST_F(VideoStreamRegistryTest, ReliableRawRosPublisherAcceptsOddSizedFrames)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_reliable_odd_dimensions"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/reliable_odd_dimensions";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage(3, 5);
  ASSERT_TRUE(publishUntil(executor, publisher, image, [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));

  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, CompressedRosPublisherAcceptsImageTransportStyleJpegFormat)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_compressed_ros"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/compressed";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kCompressedImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kCompressedImageInterfaceType);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeTransportStyleJpegImage();
  ASSERT_TRUE(publishUntil(
    executor,
    publisher,
    image,
    [&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; },
    std::chrono::seconds(5)));

  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, CompressedRosPublisherAcceptsUppercasePrimaryJpegFormatToken)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_compressed_ros_primary_jpeg"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  const std::string topic = "/camera/compressed_primary_jpeg";
  const auto info = registry.resolve(SubscriptionTargetKind::Topic, topic, kCompressedImageInterfaceType);
  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kCompressedImageInterfaceType);

  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  auto image = makeTransportStyleJpegImage();
  image.format = " JPG ; quality=95 ";
  ASSERT_TRUE(publishUntil(
    executor,
    publisher,
    image,
    [&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; },
    std::chrono::seconds(5)));

  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, ProfilingCapturesRawRosStreamActivity)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_profiled_raw_ros"));
  FakeRoomConnection room_connection;
  VideoProfilingRegistry profiling_registry(node->get_logger(), makeProfilingConfig());
  VideoStreamRegistry registry(*node, room_connection, nullptr, &profiling_registry);

  const std::string topic = "/camera/profiled_raw";
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeRgbImage();
  ASSERT_TRUE(publishUntil(executor, publisher, image, [&room_connection]() {
    return room_connection.state->published_video_track_names.size() == 1U;
  }));

  const auto summaries = profiling_registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_GE(summary.frames_in, 1U);
  EXPECT_GE(summary.frames_sampled, 1U);
  EXPECT_GE(summary.frames_captured, 1U);
  EXPECT_EQ(summary.track_publish_count, 1U);
  EXPECT_TRUE(summary.push_to_appsrc_ms.hasSamples());
  EXPECT_TRUE(summary.sample_callback_ms.hasSamples());
  EXPECT_TRUE(summary.sample_unpack_ms.hasSamples());
  EXPECT_TRUE(summary.frame_sink_ms.hasSamples());
  EXPECT_TRUE(summary.publisher_handle_frame_ms.hasSamples());
}

TEST_F(VideoStreamRegistryTest, ProfilingCapturesCompressedRosStreamActivity)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_profiled_compressed_ros"));
  FakeRoomConnection room_connection;
  VideoProfilingRegistry profiling_registry(node->get_logger(), makeProfilingConfig());
  VideoStreamRegistry registry(*node, room_connection, nullptr, &profiling_registry);

  const std::string topic = "/camera/profiled_compressed";
  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, topic, kCompressedImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&publisher]() { return publisher->get_subscription_count() == 1U; }));

  const auto image = makeTransportStyleJpegImage();
  ASSERT_TRUE(publishUntil(
    executor,
    publisher,
    image,
    [&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; },
    std::chrono::seconds(5)));

  const auto summaries = profiling_registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_GE(summary.frames_in, 1U);
  EXPECT_GE(summary.frames_sampled, 1U);
  EXPECT_GE(summary.frames_captured, 1U);
  EXPECT_TRUE(summary.push_to_appsrc_ms.hasSamples());
}

TEST_F(VideoStreamRegistryTest, ConfiguredSourceStartIsIdempotentAndStopUnpublishesPublishedTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_configured_source"));
  FakeRoomConnection room_connection;
  const auto stream_config = makeConfiguredSourceConfig({"/sources/front"});
  VideoStreamRegistry registry(*node, room_connection, nullptr, nullptr, &stream_config);

  const std::string source_name = "/sources/front";
  const auto info = registry.resolve(SubscriptionTargetKind::ConfiguredSource, source_name);

  registry.start(SubscriptionTargetKind::ConfiguredSource, source_name);
  registry.start(SubscriptionTargetKind::ConfiguredSource, source_name);

  ASSERT_TRUE(
    waitUntil([&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; }));
  EXPECT_EQ(room_connection.state->published_video_track_names, (std::vector<std::string>{info.track_name}));

  registry.stop(SubscriptionTargetKind::ConfiguredSource, source_name);

  EXPECT_EQ(room_connection.state->unpublished_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, ProfilingCapturesConfiguredSourceActivity)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_profiled_configured_source"));
  FakeRoomConnection room_connection;
  VideoProfilingRegistry profiling_registry(node->get_logger(), makeProfilingConfig());
  const auto stream_config = makeConfiguredSourceConfig({"/sources/profiled_front"});
  VideoStreamRegistry registry(*node, room_connection, nullptr, &profiling_registry, &stream_config);

  const std::string source_name = "/sources/profiled_front";

  registry.start(SubscriptionTargetKind::ConfiguredSource, source_name);
  ASSERT_TRUE(
    waitUntil([&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; }));

  std::optional<std::vector<VideoStreamProfileSummary>> captured_summaries;
  const bool summary_ready = waitUntil([&profiling_registry, &captured_summaries]() {
    if (captured_summaries.has_value()) {
      return true;
    }
    const auto summaries = profiling_registry.takeSummaries();
    if (!summaries.empty()) {
      captured_summaries = summaries;
      return true;
    }
    return false;
  });
  ASSERT_TRUE(summary_ready);
  ASSERT_EQ(captured_summaries->size(), 1U);
  const auto & summary = captured_summaries->front();
  EXPECT_GT(summary.frames_in, 0U);
  EXPECT_GT(summary.frames_sampled, 0U);
  EXPECT_GT(summary.frames_captured, 0U);
  EXPECT_TRUE(summary.sample_callback_ms.hasSamples());
}

TEST_F(VideoStreamRegistryTest, ShutdownUnpublishesActiveTracksAndIsIdempotent)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_shutdown"));
  FakeRoomConnection room_connection;
  const auto stream_config = makeConfiguredSourceConfig({"/sources/shutdown"});
  VideoStreamRegistry registry(*node, room_connection, nullptr, nullptr, &stream_config);

  const std::string source_name = "/sources/shutdown";
  const auto info = registry.resolve(SubscriptionTargetKind::ConfiguredSource, source_name);
  registry.start(SubscriptionTargetKind::ConfiguredSource, source_name);

  ASSERT_TRUE(
    waitUntil([&room_connection]() { return room_connection.state->published_video_track_names.size() == 1U; }));

  registry.shutdown();
  registry.shutdown();

  EXPECT_EQ(room_connection.state->unpublished_video_track_names, (std::vector<std::string>{info.track_name}));
}

TEST_F(VideoStreamRegistryTest, StartRejectsUnsupportedRosInterfaceType)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_unsupported_interface_type"));
  FakeRoomConnection room_connection;
  VideoStreamRegistry registry(*node, room_connection);

  try {
    registry.start(SubscriptionTargetKind::Topic, "/camera/unsupported_interface_type", "sensor_msgs/msg/PointCloud2");
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "ROS topic is not a supported video type.");
  }
}

TEST_F(VideoStreamRegistryTest, StartRejectsNewStreamsAfterShutdown)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_shutdown_reject"));
  FakeRoomConnection room_connection;
  const auto stream_config = makeConfiguredSourceConfig({"/sources/shutdown"});
  VideoStreamRegistry registry(*node, room_connection, nullptr, nullptr, &stream_config);

  const std::string source_name = "/sources/shutdown";

  registry.shutdown();

  try {
    registry.start(SubscriptionTargetKind::ConfiguredSource, source_name);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), "Video stream registry is shut down.");
  }
}

TEST_F(VideoStreamRegistryTest, PerStreamPublishConfigIsAppliedToEachPublishedTrack)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("video_stream_registry_publish_config"));
  FakeRoomConnection room_connection;
  const std::string first_topic = "/camera/publish_config/one";
  const std::string second_topic = "/camera/publish_config/two";
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();
  stream_config.ros_topic_rules.push_back(
    {first_topic, "first_publish_config", "", {VideoPublishCodec::H264, 900000, 24.0, VideoPublishSimulcast::Enabled}});
  stream_config.ros_topic_rules.push_back(
    {second_topic,
     "second_publish_config",
     "",
     {VideoPublishCodec::Vp9, 250000, 12.0, VideoPublishSimulcast::Disabled}});
  VideoStreamRegistry registry(*node, room_connection, nullptr, nullptr, &stream_config);

  const auto first_info = registry.resolve(SubscriptionTargetKind::Topic, first_topic, kImageInterfaceType);
  const auto second_info = registry.resolve(SubscriptionTargetKind::Topic, second_topic, kImageInterfaceType);

  auto first_publisher = node->create_publisher<sensor_msgs::msg::Image>(first_topic, rclcpp::QoS(10));
  auto second_publisher = node->create_publisher<sensor_msgs::msg::Image>(second_topic, rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  registry.start(SubscriptionTargetKind::Topic, first_topic, kImageInterfaceType);
  registry.start(SubscriptionTargetKind::Topic, second_topic, kImageInterfaceType);
  ASSERT_TRUE(spinUntil(executor, [&first_publisher]() { return first_publisher->get_subscription_count() == 1U; }));
  ASSERT_TRUE(spinUntil(executor, [&second_publisher]() { return second_publisher->get_subscription_count() == 1U; }));

  ASSERT_TRUE(publishUntil(executor, first_publisher, makeRgbImage(2, 2), [&room_connection]() {
    return room_connection.state->published_video_configs.size() == 1U;
  }));
  ASSERT_TRUE(publishUntil(executor, second_publisher, makeRgbImage(4, 4), [&room_connection]() {
    return room_connection.state->published_video_configs.size() == 2U;
  }));

  EXPECT_EQ(
    room_connection.state->published_video_track_names,
    (std::vector<std::string>{first_info.track_name, second_info.track_name}));

  ASSERT_EQ(room_connection.state->published_video_configs.size(), 2U);
  const auto & first_config = room_connection.state->published_video_configs[0];
  const auto & second_config = room_connection.state->published_video_configs[1];
  EXPECT_EQ(first_config.codec, stream_config.ros_topic_rules[1].publish_config.codec);
  EXPECT_EQ(first_config.max_bitrate_bps, stream_config.ros_topic_rules[1].publish_config.max_bitrate_bps);
  EXPECT_DOUBLE_EQ(first_config.max_framerate, stream_config.ros_topic_rules[1].publish_config.max_framerate);
  EXPECT_EQ(first_config.simulcast, stream_config.ros_topic_rules[1].publish_config.simulcast);
  EXPECT_EQ(second_config.codec, stream_config.ros_topic_rules[2].publish_config.codec);
  EXPECT_EQ(second_config.max_bitrate_bps, stream_config.ros_topic_rules[2].publish_config.max_bitrate_bps);
  EXPECT_DOUBLE_EQ(second_config.max_framerate, stream_config.ros_topic_rules[2].publish_config.max_framerate);
  EXPECT_EQ(second_config.simulcast, stream_config.ros_topic_rules[2].publish_config.simulcast);
}

}  // namespace livekit_ros2_bridge
