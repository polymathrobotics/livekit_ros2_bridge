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
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <future>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "utils/pipeline_failure_handler.hpp"
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
    [](livekit::EncodedVideoSource::Frame) {},
    []() {},
    {},
  };
}

StreamSpec makePassthroughSpec(const std::string & source_fragment)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", source_fragment, "", true};
  spec.publish_options.video_codec = livekit::VideoCodec::H264;
  spec.publish_options.video_encoder = livekit::VideoEncoderBackend::PreEncoded;
  spec.publish_options.simulcast = false;
  return spec;
}

// A huge GOP means every keyframe after the first one was requested.
constexpr char kOpenH264Source[] =
  "videotestsrc is-live=true ! video/x-raw,width=64,height=48,framerate=60/1 ! openh264enc gop-size=100000";

template <typename Predicate>
bool waitUntil(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

// Records formatted log lines (still forwarding them) so tests can assert on log events.
class ScopedLogCapture final
{
public:
  ScopedLogCapture()
  {
    std::lock_guard<std::mutex> lock(mutex());
    lines().clear();
    previousHandler() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&ScopedLogCapture::handle);
  }

  ~ScopedLogCapture()
  {
    std::lock_guard<std::mutex> lock(mutex());
    rcutils_logging_set_output_handler(previousHandler());
  }

  ScopedLogCapture(const ScopedLogCapture &) = delete;
  ScopedLogCapture & operator=(const ScopedLogCapture &) = delete;
  ScopedLogCapture(ScopedLogCapture &&) = delete;
  ScopedLogCapture & operator=(ScopedLogCapture &&) = delete;

  // Returns the first captured line that contains every fragment, or an empty string.
  std::string findLine(std::initializer_list<const char *> fragments) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    for (const auto & line : lines()) {
      bool matches = true;
      for (const char * fragment : fragments) {
        matches = matches && line.find(fragment) != std::string::npos;
      }
      if (matches) {
        return line;
      }
    }
    return "";
  }

private:
  static void handle(
    const rcutils_log_location_t * location,
    int severity,
    const char * name,
    rcutils_time_point_value_t timestamp,
    const char * format,
    va_list * args)
  {
    va_list arguments;
    va_copy(arguments, *args);
    char buffer[4096];
    std::vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    rcutils_logging_output_handler_t previous_handler = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex());
      lines().emplace_back(buffer);
      previous_handler = previousHandler();
    }
    if (previous_handler != nullptr) {
      previous_handler(location, severity, name, timestamp, format, args);
    }
  }

  static std::mutex & mutex()
  {
    static std::mutex instance;
    return instance;
  }

  static std::vector<std::string> & lines()
  {
    static std::vector<std::string> instance;
    return instance;
  }

  static rcutils_logging_output_handler_t & previousHandler()
  {
    static rcutils_logging_output_handler_t instance = nullptr;
    return instance;
  }
};

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

// Owned by the worker in finishesWithin() so a deadlocked worker keeps it alive.
struct StreamUnderTest
{
  explicit StreamUnderTest(const StreamSpec & spec)
  : publisher(connection, spec)
  , stream(spec, publisher)
  {}

  FakeRoomConnection connection;
  TrackPublisher publisher;
  GStreamerStream stream;
};

// Runs body on a detached thread. On a timeout the thread is leaked with what it
// captured, so a deadlock fails the test instead of hanging the suite.
bool finishesWithin(std::chrono::seconds timeout, std::function<void()> body)
{
  auto finished = std::make_shared<std::promise<void>>();
  std::future<void> finished_future = finished->get_future();
  std::thread([body = std::move(body), finished]() mutable {
    body();
    // Release the captures first so the caller holds the last reference.
    body = nullptr;
    finished->set_value();
  }).detach();
  return finished_future.wait_for(timeout) == std::future_status::ready;
}

// A source that fails while the state change runs (missing file) is delivered
// through the sync bus handler on the calling thread. The failure path must not
// take the stream mutex_: start() has to return or throw promptly, and close()
// has to complete afterwards.
void expectStartWithUnavailableSourceDoesNotWedge(const StreamSpec & spec)
{
  auto under_test = std::make_shared<StreamUnderTest>(spec);
  const auto start_stream = [under_test]() {
    try {
      under_test->stream.start();
    } catch (const std::exception &) {
    }
  };
  const auto close_stream = [under_test]() { under_test->stream.close(); };

  EXPECT_TRUE(finishesWithin(std::chrono::seconds(10), start_stream)) << "start() did not finish";
  EXPECT_TRUE(finishesWithin(std::chrono::seconds(10), close_stream)) << "close() did not finish";
}

TEST_F(StreamTest, StartWithUnavailableSourceDoesNotWedge)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "filesrc location=/nonexistent/lkros_missing_source.raw", ""};

  expectStartWithUnavailableSourceDoesNotWedge(spec);
}

TEST_F(StreamTest, PassthroughStartWithUnavailableSourceDoesNotWedge)
{
  expectStartWithUnavailableSourceDoesNotWedge(
    makePassthroughSpec("filesrc location=/nonexistent/lkros_missing_source.h264"));
}

TEST_F(StreamTest, PassthroughPipelineDeliversEncodedFramesAndHonoursKeyFrameRequests)
{
  constexpr std::size_t kRequestAtFrame = 10U;
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<livekit::EncodedVideoSource::Frame> frames;
  GStreamerPipeline * pipeline_handle = nullptr;
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  callbacks.on_encoded_frame = [&](livekit::EncodedVideoSource::Frame frame) {
    std::lock_guard<std::mutex> lock(mutex);
    frames.push_back(std::move(frame));
    // Requesting from inside the sample callback must not deadlock.
    if (frames.size() == kRequestAtFrame) {
      pipeline_handle->requestKeyFrame();
    }
    condition.notify_all();
  };
  GStreamerPipeline pipeline(callbacks);
  pipeline_handle = &pipeline;

  pipeline.requestKeyFrame();
  pipeline.start(buildPassthroughPipelineDescription(kOpenH264Source), false, PipelineOutput::EncodedH264);
  std::unique_lock<std::mutex> lock(mutex);
  const bool requested_key_frame_arrived = condition.wait_for(lock, std::chrono::seconds(10), [&]() {
    for (std::size_t index = kRequestAtFrame; index < frames.size(); ++index) {
      if (frames[index].is_keyframe) {
        return true;
      }
    }
    return false;
  });
  lock.unlock();
  pipeline.stop();
  pipeline.requestKeyFrame();

  EXPECT_TRUE(requested_key_frame_arrived);
  ASSERT_GE(frames.size(), kRequestAtFrame);
  EXPECT_TRUE(frames.front().is_keyframe);
  EXPECT_EQ(frames.front().width, 64U);
  EXPECT_EQ(frames.front().height, 48U);
  EXPECT_FALSE(frames.front().data.empty());
  for (std::size_t index = 1U; index < kRequestAtFrame; ++index) {
    EXPECT_FALSE(frames[index].is_keyframe) << "frame " << index;
    EXPECT_GT(frames[index].timestamp_us, frames[index - 1U].timestamp_us) << "frame " << index;
  }
}

TEST_F(StreamTest, PassthroughStreamPublishesEncodedSource)
{
  StreamSpec spec = makePassthroughSpec(kOpenH264Source);
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool published = waitUntil([&]() { return connection.state->publishedVideoTrackCount() == 1U; });
  stream.close();

  ASSERT_TRUE(published);
  ASSERT_EQ(connection.state->published_video_options.size(), 1U);
  EXPECT_EQ(connection.state->published_video_options.front().video_encoder, livekit::VideoEncoderBackend::PreEncoded);
  ASSERT_EQ(connection.state->published_video_sources.size(), 1U);
  EXPECT_NE(
    std::dynamic_pointer_cast<livekit::EncodedVideoSource>(connection.state->published_video_sources.front()), nullptr);
}

TEST_F(StreamTest, PassthroughStreamExplainsNonH264Source)
{
  ScopedLogCapture logs;
  // A static vp8enc ! h264parse link already fails to parse (and config validation
  // rejects it); the demuxer's dynamic pad fails at runtime like an RTSP camera would.
  StreamSpec spec =
    makePassthroughSpec("videotestsrc is-live=true ! vp8enc deadline=1 ! matroskamux streamable=true ! matroskademux");
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool hint_logged = waitUntil([&]() {
    return !logs.findLine({"video_stream_pipeline_recovery_scheduled", "passthrough supports H.264 only"}).empty();
  });
  stream.close();

  ASSERT_TRUE(hint_logged);
  const std::string line = logs.findLine({"video_stream_pipeline_recovery_scheduled"});
  EXPECT_NE(line.find("offered_caps=\"video/x-vp8"), std::string::npos) << line;
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 0U);
}

TEST_F(StreamTest, PassthroughStreamSkipsToRequestedKeyFrameAfterFallingBehind)
{
  ScopedLogCapture logs;
  StreamSpec spec = makePassthroughSpec(kOpenH264Source);
  FakeRoomConnection connection;
  bool publish_started = false;
  std::promise<void> publish_started_promise;
  std::future<void> publish_started_future = publish_started_promise.get_future();
  std::promise<void> release_publish_promise;
  std::shared_future<void> release_publish_future = release_publish_promise.get_future().share();
  // Blocking the first publish stalls the sample thread while the source keeps producing.
  connection.state->publish_video_track_hook = [&](const std::string &) {
    if (!publish_started) {
      publish_started = true;
      publish_started_promise.set_value();
    }
    release_publish_future.wait();
  };
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool publish_blocked = publish_started_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  // At 60 fps the bridge queue (8 buffers) overruns well within this stall.
  std::this_thread::sleep_for(std::chrono::milliseconds(750));
  release_publish_promise.set_value();
  const bool resumed = waitUntil([&]() { return !logs.findLine({"video_stream_passthrough_resumed"}).empty(); });
  stream.close();

  ASSERT_TRUE(publish_blocked);
  EXPECT_FALSE(logs.findLine({"video_stream_passthrough_fell_behind", "stream_key=other_video:test"}).empty());
  ASSERT_TRUE(resumed);
  const std::string resumed_line = logs.findLine({"video_stream_passthrough_resumed"});
  EXPECT_EQ(resumed_line.find("discarded_frames=0"), std::string::npos) << resumed_line;
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 1U);
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
