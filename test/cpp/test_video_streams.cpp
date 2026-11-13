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
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "gtest/gtest.h"
#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "support/controlled_start_element.hpp"
#include "support/detached_task.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "support/video_pipeline_callbacks.hpp"
#include "utils/log_event.hpp"
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

// The interval the stream throttles its restart warnings to.
constexpr auto kLogThrottle = std::chrono::seconds(5);

// Expects `event` logged at least once, first with count=1, and at most once per throttle
// interval since `started_at`; bounding by the elapsed time keeps this right on a slow run.
void expectThrottledEvent(
  const test_support::ScopedLogCapture & logs, const char * event, std::chrono::steady_clock::time_point started_at)
{
  const std::vector<std::string> counts = logs.fieldValues({event}, "count");
  const auto elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at);
  const std::size_t max_lines = 1U + static_cast<std::size_t>(elapsed / kLogThrottle);
  ASSERT_FALSE(counts.empty()) << event << " was not logged.";
  EXPECT_EQ(counts.front(), "1") << event;
  EXPECT_LE(counts.size(), max_lines) << event << " after " << elapsed.count() << " ms";
}

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
  GStreamerPipeline pipeline(test_support::makeNoOpVideoPipelineCallbacks());

  expectStartErrorContains(
    pipeline, "videotestsrc is-live=true ! fakesink name=bridge_video_sink", false, "must be a GstAppSink");
}

TEST_F(StreamTest, PipelineStartRejectsNamedNonAppSrcWhenRequired)
{
  GStreamerPipeline pipeline(test_support::makeNoOpVideoPipelineCallbacks());

  expectStartErrorContains(
    pipeline,
    "videotestsrc is-live=true ! identity name=bridge_video_src ! appsink name=bridge_video_sink",
    true,
    "must be a GstAppSrc");
}

TEST_F(StreamTest, PipelineStartCapturesRequiredAppSrcHandle)
{
  GStreamerPipeline pipeline(test_support::makeNoOpVideoPipelineCallbacks());

  pipeline.start("appsrc name=bridge_video_src is-live=true ! appsink name=bridge_video_sink", true);

  EXPECT_NE(pipeline.appsrc(), nullptr);
  pipeline.stop();
}

TEST_F(StreamTest, PipelineContainsNonStandardCallbackExceptions)
{
  std::mutex error_mutex;
  std::string capture_error;
  std::string failure_reason;
  PipelineCallbacks callbacks = test_support::makeNoOpVideoPipelineCallbacks();
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

// A static vp8enc link already fails to parse (and config validation rejects it); the
// demuxer's dynamic pad fails at runtime like an RTSP camera would.
TEST_F(StreamTest, PassthroughPipelineReportsNonH264SourceAsCapsMismatch)
{
  std::mutex mismatch_mutex;
  std::string offered_caps;
  std::atomic<bool> mismatch_reported{false};
  std::atomic<bool> failure_reported{false};
  PipelineCallbacks callbacks = test_support::makeNoOpVideoPipelineCallbacks();
  callbacks.on_caps_mismatch = [&](const std::string &, const std::string & caps) {
    const std::lock_guard<std::mutex> lock(mismatch_mutex);
    if (!mismatch_reported.exchange(true)) {
      offered_caps = caps;
    }
  };
  callbacks.on_failed = [&](const std::string &) { failure_reported = true; };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(
    buildPassthroughPipelineDescription(
      "videotestsrc is-live=true ! vp8enc deadline=1 ! matroskamux streamable=true ! matroskademux"),
    false,
    PipelineOutput::EncodedH264);
  const bool reported = test_support::waitUntil([&]() { return mismatch_reported.load(); }, std::chrono::seconds(10));
  pipeline.stop();

  ASSERT_TRUE(reported);
  EXPECT_EQ(offered_caps.rfind("video/x-vp8", 0U), 0U) << offered_caps;
  EXPECT_FALSE(failure_reported.load());
}

// Holds the first frame, as a stalled consumer would, until the bridge queue overruns. The
// pipeline must contain and log what on_fell_behind throws.
TEST_F(StreamTest, PassthroughPipelineReportsFellBehindWhenItsQueueFills)
{
  test_support::ScopedLogCapture logs;
  std::atomic<bool> fell_behind{false};
  const auto reported_fell_behind = [&]() { return fell_behind.load(); };
  PipelineCallbacks callbacks = test_support::makeNoOpVideoPipelineCallbacks();
  callbacks.on_encoded_frame = [&](livekit::EncodedVideoSource::Frame) {
    (void)test_support::waitUntil(reported_fell_behind, std::chrono::seconds(10));
  };
  callbacks.on_fell_behind = [&]() {
    fell_behind = true;
    throw std::runtime_error("simulated fell behind failure");
  };
  GStreamerPipeline pipeline(callbacks);

  // datarate stamps each 16-byte buffer 0.1 s after the last, so ten of them fill the queue.
  pipeline.start(
    buildPassthroughPipelineDescription(
      "fakesrc format=time datarate=160 sizetype=fixed sizemax=16 filltype=zero"
      " ! capsfilter caps=video/x-h264,stream-format=byte-stream,alignment=au,width=64,height=48"),
    false,
    PipelineOutput::EncodedH264);
  const bool reported = test_support::waitUntil(reported_fell_behind, std::chrono::seconds(10));
  pipeline.stop();

  EXPECT_TRUE(reported);
  EXPECT_GE(
    logs.countLines(
      {"video_pipeline_callback_failed", "callback=queue_overrun", "error=simulated fell behind failure"}),
    1U);
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

// A source that fails again after every restart, first by reaching EOS and then by
// failing to start at all, keeps being restarted, and its warnings are throttled.
TEST_F(StreamTest, SourceThatStaysBrokenKeepsBeingRestarted)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::Normal);
  // The mode is process-wide, so put it back for later tests.
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  StreamSpec spec = makeOtherSpec();
  // About 100 ms of video, then EOS: well after the restart that started it has returned.
  spec.input = OtherInput{
    "test",
    std::string("videotestsrc is-live=true num-buffers=4 ! video/x-raw,width=64,height=48,framerate=30/1 ! ") +
      ControlledStartElement::kFactoryName,
    ""};
  test_support::ScopedLogCapture logs;
  const auto harness = std::make_shared<OtherStreamHarness>(spec);
  const std::size_t attempts_before = ControlledStartElement::startAttempts();
  const auto attempts = [attempts_before]() { return ControlledStartElement::startAttempts() - attempts_before; };
  const auto start_stream = [harness]() { harness->stream.start(); };
  const auto close_stream = [harness]() { harness->stream.close(); };

  const auto started_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
  // The initial start and two restarts after EOS.
  const bool restarted_after_eos = test_support::waitUntil([&]() { return attempts() >= 3U; }, std::chrono::seconds(5));
  // From the next restart on, every start() throws.
  ControlledStartElement::setMode(ControlledStartElement::Mode::FailStart);
  const bool retried_failed_restart =
    restarted_after_eos && test_support::waitUntil([&]() { return attempts() >= 6U; }, std::chrono::seconds(5));
  EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";

  EXPECT_TRUE(restarted_after_eos) << "start attempts: " << attempts();
  EXPECT_TRUE(retried_failed_restart) << "start attempts: " << attempts();
  // Three EOS failures and at least two failed restarts, logged as one line each unless
  // the run took longer than a throttle interval.
  expectThrottledEvent(logs, "video_stream_pipeline_recovery_scheduled", started_at);
  expectThrottledEvent(logs, "video_stream_restart_failed", started_at);
}

// A failure that arrives while a restart's start() runs, without making it throw,
// lands while the restart callback runs. It must still lead to another restart.
TEST_F(StreamTest, FailureDuringRestartLeadsToAnotherRestart)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::PostErrorOnStart);
  // The mode is process-wide, so put it back for later tests.
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  StreamSpec spec = makeOtherSpec();
  spec.input =
    OtherInput{"test", std::string("videotestsrc is-live=true ! ") + ControlledStartElement::kFactoryName, ""};
  const auto harness = std::make_shared<OtherStreamHarness>(spec);
  const std::size_t attempts_before = ControlledStartElement::startAttempts();
  const auto attempts = [attempts_before]() { return ControlledStartElement::startAttempts() - attempts_before; };
  const auto start_stream = [harness]() { harness->stream.start(); };
  const auto close_stream = [harness]() { harness->stream.close(); };

  // The first start's failure schedules a restart; that restart's failure is the
  // one that lands during the callback.
  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
  const bool restarted_after_failed_restart =
    test_support::waitUntil([&]() { return attempts() >= 3U; }, std::chrono::seconds(5));
  EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";

  EXPECT_TRUE(restarted_after_failed_restart) << "start attempts: " << attempts();
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
