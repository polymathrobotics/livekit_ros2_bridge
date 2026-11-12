// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "gtest/gtest.h"
#include "livekit/video_frame.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "support/controlled_start_element.hpp"
#include "support/detached_task.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "utils/log_event.hpp"
#include "utils/pipeline_failure_handler.hpp"
#include "utils/scope_exit.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/gstreamer_resources.hpp"
#include "video/gstreamer_stream.hpp"
#include "video/pipeline_description.hpp"
#include "video/ros_stream.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::VideoSource; see ScopedLiveKitInit.
const test_support::ScopedLiveKitInit kLiveKitInit;

// How long a call that could deadlock gets before the test gives up on it.
constexpr auto kHangTimeout = std::chrono::seconds(10);

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

class StreamTest : public test_support::RclcppTestSuite
{
protected:
  static void SetUpTestSuite()
  {
    ensureGStreamerInitialized();
    RclcppTestSuite::SetUpTestSuite();
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

TEST_F(StreamTest, PipelineContainsNonStandardCallbackExceptions)
{
  std::mutex error_mutex;
  std::string capture_error;
  std::string failure_reason;
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  // on_capture_failed also throws, so the exception reaches the new_sample catch;
  // the failed sample makes the pipeline post an error, which on_failed throws on.
  callbacks.on_frame = [](const livekit::VideoFrame &, std::int64_t) { throw 42; };
  callbacks.on_capture_failed = [&](const std::string & error) {
    const std::lock_guard<std::mutex> lock(error_mutex);
    if (capture_error.empty()) {
      capture_error = error;
    }
    throw 42;
  };
  callbacks.on_failed = [&](const std::string & reason) {
    const std::lock_guard<std::mutex> lock(error_mutex);
    if (failure_reason.empty()) {
      failure_reason = reason;
    }
    throw 42;
  };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(buildPipelineDescription("videotestsrc is-live=true ! video/x-raw,width=64,height=48", ""), false);
  const bool reported = test_support::waitUntil(
    [&]() {
      const std::lock_guard<std::mutex> lock(error_mutex);
      return !failure_reason.empty();
    },
    std::chrono::seconds(10));
  pipeline.stop();

  EXPECT_EQ(capture_error, std::string(kUnknownExceptionValue));
  EXPECT_TRUE(reported);
  EXPECT_NE(failure_reason, "eos");
}

TEST_F(StreamTest, PipelineFailureHandlerCoalescesPendingFailures)
{
  std::atomic<int> callback_count{0};
  utils::PipelineFailureHandler handler(std::chrono::seconds(10), [&]() { ++callback_count; });

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
  utils::PipelineFailureHandler handler(std::chrono::milliseconds::zero(), [&]() {
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
  std::unique_ptr<utils::PipelineFailureHandler> handler;
  int callback_count = 0;
  bool reschedule_checked = false;
  bool reschedule_accepted = true;
  handler = std::make_unique<utils::PipelineFailureHandler>(std::chrono::milliseconds::zero(), [&]() {
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
  utils::PipelineFailureHandler handler(std::chrono::seconds(10), [&]() { ++callback_count; });

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

// Owns an other-source stream and what it borrows, so a call that never returns can keep
// all of it alive on its abandoned thread.
struct OtherStreamHarness
{
  explicit OtherStreamHarness(const StreamSpec & spec)
  : track_publisher(connection, spec)
  , stream(spec, track_publisher)
  {}

  FakeRoomConnection connection;
  TrackPublisher track_publisher;
  GStreamerStream stream;
};

// A source that fails while the state change runs (missing file) is delivered
// through the sync bus handler on the calling thread. The failure path must not
// take the stream mutex_: start() has to return or throw promptly, and close()
// has to complete afterwards.
TEST_F(StreamTest, StartWithUnavailableSourceDoesNotWedge)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "filesrc location=/nonexistent/lkros_missing_source.raw", ""};
  const auto harness = std::make_shared<OtherStreamHarness>(spec);

  const auto start_stream = [harness]() {
    try {
      harness->stream.start();
    } catch (const std::exception &) {
    }
  };
  const auto close_stream = [harness]() { harness->stream.close(); };

  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
  EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";
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

// Owns a ROS image stream and everything it borrows, so a call that never returns can keep
// all of it alive on its abandoned thread.
struct RosImageStreamHarness
{
  RosImageStreamHarness(const std::string & node_name, const StreamSpec & spec)
  : node(std::make_shared<rclcpp::Node>(node_name))
  , track_publisher(connection, spec)
  , stream(std::make_shared<RosStream>(*node, spec, nullptr, track_publisher))
  {
    executor.add_node(node);
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::executors::SingleThreadedExecutor executor;
  FakeRoomConnection connection;
  TrackPublisher track_publisher;
  std::shared_ptr<RosStream> stream;
};

sensor_msgs::msg::Image makeRgbImage()
{
  sensor_msgs::msg::Image image;
  image.encoding = sensor_msgs::image_encodings::RGB8;
  image.width = 64U;
  image.height = 48U;
  image.step = 192U;
  image.data.resize(image.step * image.height);
  return image;
}

// A transform that reports an error while the pipeline starts delivers it on the ROS
// callback's thread, which holds the stream mutex_. The callback must still return.
TEST_F(StreamTest, RosImageWithFailingTransformDoesNotWedge)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::PostErrorOnStart);
  // The mode is process-wide, so put it back for later tests.
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  const std::string topic = "/video_pipeline/failing_transform_image";
  StreamSpec spec = makeRosTopicSpec(topic, imageInterfaceType());
  std::get<RosInput>(spec.input).transform_fragment = ControlledStartElement::kFactoryName;
  auto harness = std::make_shared<RosImageStreamHarness>("video_pipeline_ros_failing_transform_test", spec);
  const auto image_publisher = harness->node->create_publisher<sensor_msgs::msg::Image>(topic, 10);
  harness->stream->start();
  ASSERT_TRUE(
    test_support::spinUntil(harness->executor, [&]() { return image_publisher->get_subscription_count() == 1U; }));

  const std::size_t attempts_before = ControlledStartElement::startAttempts();
  image_publisher->publish(makeRgbImage());

  const auto spin_until_pipeline_starts = [harness, attempts_before]() {
    test_support::spinUntil(
      harness->executor, [&]() { return ControlledStartElement::startAttempts() > attempts_before; });
  };

  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, spin_until_pipeline_starts))
    << "The ROS image callback did not return.";
  EXPECT_GT(ControlledStartElement::startAttempts(), attempts_before);
}

// A transform that fails on every start makes each ROS frame rebuild the pipeline, but the
// stream logs that its pipeline failed once per throttle interval, not once per frame.
TEST_F(StreamTest, RosImageWithFailingTransformThrottlesItsWarning)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::PostErrorOnStart);
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  test_support::ScopedLogCapture logs;
  const std::string topic = "/video_pipeline/throttled_failing_transform_image";
  StreamSpec spec = makeRosTopicSpec(topic, imageInterfaceType());
  std::get<RosInput>(spec.input).transform_fragment = ControlledStartElement::kFactoryName;
  auto harness = std::make_shared<RosImageStreamHarness>("video_pipeline_ros_throttled_transform_test", spec);
  const auto image_publisher = harness->node->create_publisher<sensor_msgs::msg::Image>(topic, 10);
  harness->stream->start();
  ASSERT_TRUE(
    test_support::spinUntil(harness->executor, [&]() { return image_publisher->get_subscription_count() == 1U; }));

  // One frame at a time, so the subscription's depth of one never drops a frame.
  constexpr std::size_t kFrameCount = 3U;
  const std::size_t attempts_before = ControlledStartElement::startAttempts();
  const auto publish_frames = [harness, image_publisher]() {
    for (std::size_t frame = 0U; frame < kFrameCount; ++frame) {
      const std::size_t attempts_before_frame = ControlledStartElement::startAttempts();
      image_publisher->publish(makeRgbImage());
      test_support::spinUntil(
        harness->executor, [&]() { return ControlledStartElement::startAttempts() > attempts_before_frame; });
    }
  };

  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, publish_frames)) << "A ROS image callback did not return.";
  ASSERT_EQ(ControlledStartElement::startAttempts() - attempts_before, kFrameCount);
  const std::string stream_key_field = "stream_key=" + spec.stream_key;
  EXPECT_EQ(logs.countLines({"video_stream_pipeline_recovery_disabled", stream_key_field.c_str()}), 1U);
  EXPECT_EQ(
    logs.fieldValues({"video_stream_pipeline_recovery_disabled", stream_key_field.c_str()}, "count"),
    std::vector<std::string>{"1"});
}

}  // namespace

}  // namespace livekit_ros2_bridge::video
