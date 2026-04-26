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
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "livekit/video_frame.h"
#include "ros_test_support.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/gstreamer_resources.hpp"
#include "video/gstreamer_stream.hpp"
#include "video/pipeline_failure_handler.hpp"
#include "video/ros_stream.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

StreamSpec makeOtherSpec()
{
  StreamSpec spec;
  spec.stream_key = "other_video:test";
  spec.track_name = "lkros.video.other.test";
  spec.input = OtherInput{"test", "", ""};
  return spec;
}

StreamSpec makeRosTopicSpec(const std::string & topic, const char * interface_type)
{
  StreamConfig config = makeDefaultConfig();
  config.ros_topic_rules.front().rule_id = "test";
  StreamSpec spec = resolveRosTopicSpec(config, topic, interface_type);
  spec.track_name = "lkros.video.test";
  return spec;
}

const char * imageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::Image>();
}

const char * compressedImageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>();
}

PipelineCallbacks makeNoOpPipelineCallbacks()
{
  return PipelineCallbacks{
    []() { return false; },
    [](const livekit::VideoFrame &, std::int64_t) {},
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  };
}

void expectStartErrorContains(
  GStreamerPipeline & pipeline, const std::string & description, bool require_appsrc, const char * fragment)
{
  try {
    pipeline.start(description, require_appsrc);
    FAIL() << "Expected start to throw an error containing '" << fragment << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(fragment), std::string::npos) << "actual error: " << error.what();
  }
}

class StreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ensureGStreamerInitialized();
    static test_support::ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(StreamTest, PipelineStartRejectsNamedNonAppSink)
{
  GStreamerPipeline pipeline(makeNoOpPipelineCallbacks());

  expectStartErrorContains(
    pipeline, "videotestsrc is-live=true ! fakesink name=bridge_video_sink", false, "must be a GstAppSink");
}

TEST_F(StreamTest, PipelineStartRejectsNamedNonAppSrcWhenRequired)
{
  GStreamerPipeline pipeline(makeNoOpPipelineCallbacks());

  expectStartErrorContains(
    pipeline,
    "videotestsrc is-live=true ! identity name=bridge_video_src ! appsink name=bridge_video_sink",
    true,
    "must be a GstAppSrc");
}

TEST_F(StreamTest, PipelineStartCapturesRequiredAppSrcHandle)
{
  GStreamerPipeline pipeline(makeNoOpPipelineCallbacks());

  pipeline.start("appsrc name=bridge_video_src is-live=true ! appsink name=bridge_video_sink", true);

  EXPECT_NE(pipeline.appsrc(), nullptr);
  pipeline.stop();
}

TEST_F(StreamTest, PipelineFailureHandlerCoalescesPendingFailures)
{
  std::atomic<int> callback_count{0};
  PipelineFailureHandler handler(std::chrono::seconds(10), [&]() { ++callback_count; });

  EXPECT_TRUE(handler.schedule());
  EXPECT_FALSE(handler.schedule());
  handler.cancelPending();
  EXPECT_TRUE(handler.schedule());
  handler.close();

  EXPECT_EQ(callback_count.load(), 0);
}

TEST_F(StreamTest, PipelineFailureHandlerRunsScheduledFailure)
{
  std::mutex mutex;
  std::condition_variable condition;
  int callback_count = 0;
  PipelineFailureHandler handler(std::chrono::milliseconds::zero(), [&]() {
    std::lock_guard<std::mutex> lock(mutex);
    ++callback_count;
    condition.notify_all();
  });

  EXPECT_TRUE(handler.schedule());
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return callback_count == 1; }));
  lock.unlock();
  handler.close();

  EXPECT_EQ(callback_count, 1);
}

TEST_F(StreamTest, PipelineFailureHandlerCoalescesWhileCallbackRuns)
{
  std::mutex mutex;
  std::condition_variable condition;
  std::unique_ptr<PipelineFailureHandler> handler;
  int callback_count = 0;
  bool reschedule_checked = false;
  bool reschedule_accepted = true;
  handler = std::make_unique<PipelineFailureHandler>(std::chrono::milliseconds::zero(), [&]() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++callback_count;
    }
    const bool accepted = handler->schedule();
    {
      std::lock_guard<std::mutex> lock(mutex);
      reschedule_accepted = accepted;
      reschedule_checked = true;
      condition.notify_all();
    }
  });

  EXPECT_TRUE(handler->schedule());
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return reschedule_checked; }));
  lock.unlock();
  handler->close();

  EXPECT_EQ(callback_count, 1);
  EXPECT_FALSE(reschedule_accepted);
}

TEST_F(StreamTest, PipelineFailureHandlerCloseCancelsDelayedFailure)
{
  std::atomic<int> callback_count{0};
  PipelineFailureHandler handler(std::chrono::seconds(10), [&]() { ++callback_count; });

  EXPECT_TRUE(handler.schedule());
  handler.close();

  EXPECT_EQ(callback_count.load(), 0);
  EXPECT_FALSE(handler.schedule());
}

TEST_F(StreamTest, OtherVideoLifecycleIsIdempotent)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "videotestsrc is-live=true pattern=black", ""};

  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  stream.close();
  stream.close();
}

void expectRosTopicStreamLifecycleIsIdempotent(
  const std::string & node_name, const std::string & topic, const char * interface_type)
{
  auto node = std::make_shared<rclcpp::Node>(node_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  StreamSpec spec = makeRosTopicSpec(topic, interface_type);
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  auto stream = std::make_shared<RosStream>(*node, spec, nullptr, publisher);

  stream->start();
  stream->start();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 1U; }));

  stream->close();
  stream->close();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 0U; }));
}

TEST_F(StreamTest, RawRosTopicStreamLifecycleIsIdempotent)
{
  expectRosTopicStreamLifecycleIsIdempotent(
    "video_pipeline_ros_topic_raw_source_test", "/video_pipeline/raw_image", imageInterfaceType());
}

TEST_F(StreamTest, CompressedRosTopicStreamLifecycleIsIdempotent)
{
  expectRosTopicStreamLifecycleIsIdempotent(
    "video_pipeline_ros_topic_compressed_source_test",
    "/video_pipeline/compressed_image",
    compressedImageInterfaceType());
}

}  // namespace

}  // namespace livekit_ros2_bridge::video
