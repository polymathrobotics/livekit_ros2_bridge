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
#include <cstdio>
#include <functional>
#include <future>
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
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "support/error_on_start_element.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "support/scoped_temp_file.hpp"
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

// on_caps_mismatch stays unset, so caps failures reach on_failed.
PipelineCallbacks makeNoOpPipelineCallbacks()
{
  PipelineCallbacks callbacks;
  callbacks.is_shutdown = []() { return false; };
  callbacks.on_frame = [](const livekit::VideoFrame &, std::int64_t) {};
  callbacks.on_unpack_failed = [](const std::string &) {};
  callbacks.on_capture_failed = [](const std::string &) {};
  callbacks.on_failed = [](const std::string &) {};
  callbacks.on_encoded_frame = [](livekit::EncodedVideoSource::Frame) {};
  callbacks.on_fell_behind = []() {};
  callbacks.on_fell_behind_failed = [](const std::string &) {};
  return callbacks;
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

livekit::EncodedVideoSource::Frame makeEncodedFrame(bool is_keyframe)
{
  livekit::EncodedVideoSource::Frame frame;
  frame.is_keyframe = is_keyframe;
  frame.width = 64U;
  frame.height = 48U;
  frame.timestamp_us = 1000;
  // Annex-B start code and an IDR (type 5) or non-IDR (type 1) slice header.
  const std::uint8_t nal_header = is_keyframe ? 0x65U : 0x41U;
  frame.data = {0x00U, 0x00U, 0x00U, 0x01U, nal_header, 0x88U};
  return frame;
}

// A huge GOP means every keyframe after the first one was requested.
constexpr char kOpenH264Source[] =
  "videotestsrc is-live=true ! video/x-raw,width=64,height=48,framerate=60/1 ! openh264enc gop-size=100000";
// The bridge queue holds one second of video. Holding the sample thread twice as
// long overruns it; holding it half as long does not.
constexpr auto kQueueOverrunningStall = std::chrono::milliseconds(2000);
constexpr auto kQueueFittingStall = std::chrono::milliseconds(500);

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

// A source that fails during the state change (missing file) reports through the sync bus handler
// on the calling thread. The failure path must not take the stream mutex_, so start() must return
// or throw promptly and close() must complete afterwards.
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

// A source that keeps failing is retried every restart delay; its warnings are
// throttled instead of logged on every attempt.
TEST_F(StreamTest, RepeatedRestartFailuresAreLoggedOnce)
{
  // One 64x48 I420 frame: the source plays it, reaches EOS, and restarts.
  const test_support::ScopedTempFile source_file("lkros_restart_source_");
  {
    std::FILE * file = std::fopen(source_file.path().c_str(), "wb");
    ASSERT_NE(file, nullptr);
    const std::vector<unsigned char> frame(64U * 48U * 3U / 2U, 0x80U);
    std::fwrite(frame.data(), 1U, frame.size(), file);
    std::fclose(file);
  }
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{
    "test",
    "filesrc location=" + source_file.path() + " ! rawvideoparse width=64 height=48 format=i420 framerate=30/1",
    ""};

  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);
  stream.start();
  ASSERT_TRUE(waitUntil([&]() { return !logs.findLine({"video_stream_pipeline_recovery_scheduled"}).empty(); }));

  // From here every restart fails at once; give it several attempts.
  source_file.remove();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  stream.close();

  EXPECT_EQ(logs.countLines({"video_stream_pipeline_recovery_scheduled"}), 1U);
  EXPECT_EQ(logs.countLines({"video_stream_restart_failed"}), 1U);
}

// A failure that arrives while a restart's start() runs, without making it throw,
// lands while the restart callback runs. It must still lead to another restart.
TEST_F(StreamTest, FailureDuringRestartLeadsToAnotherRestart)
{
  test_support::ErrorOnStartElement::registerFactory();
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{
    "test", std::string("videotestsrc is-live=true ! ") + test_support::ErrorOnStartElement::kFactoryName, ""};
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);
  const std::size_t starts_before = test_support::ErrorOnStartElement::startCount();
  const auto starts = [&]() { return test_support::ErrorOnStartElement::startCount() - starts_before; };

  // The first start's failure schedules a restart; that restart's failure is the
  // one that lands during the callback.
  stream.start();
  const bool restarted_after_failed_restart = waitUntil([&]() { return starts() >= 3U; });
  stream.close();

  EXPECT_TRUE(restarted_after_failed_restart) << "starts: " << starts();
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

TEST_F(StreamTest, PassthroughPipelineReportsFellBehindCallbackFailure)
{
  std::promise<std::string> fell_behind_failed_promise;
  std::shared_future<std::string> fell_behind_failed_future = fell_behind_failed_promise.get_future().share();
  std::atomic<bool> fell_behind_failed{false};
  std::atomic<bool> capture_failed{false};
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  // Holding the sample thread fills the bridge queue until it overruns.
  callbacks.on_encoded_frame = [&](livekit::EncodedVideoSource::Frame) {
    (void)fell_behind_failed_future.wait_for(std::chrono::seconds(10));
  };
  callbacks.on_fell_behind = []() { throw std::runtime_error("simulated fell behind failure"); };
  callbacks.on_fell_behind_failed = [&](const std::string & error) {
    if (!fell_behind_failed.exchange(true)) {
      fell_behind_failed_promise.set_value(error);
    }
  };
  callbacks.on_capture_failed = [&](const std::string &) { capture_failed = true; };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(buildPassthroughPipelineDescription(kOpenH264Source), false, PipelineOutput::EncodedH264);
  const bool reported = fell_behind_failed_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  pipeline.stop();

  ASSERT_TRUE(reported);
  EXPECT_EQ(fell_behind_failed_future.get(), "simulated fell behind failure");
  EXPECT_FALSE(capture_failed.load());
}

// Anything thrown into GStreamer's C frames would terminate the process.
TEST_F(StreamTest, PassthroughPipelineContainsNonStandardFellBehindException)
{
  std::atomic<bool> frame_held{false};
  std::atomic<bool> fell_behind{false};
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  callbacks.on_encoded_frame = [&](livekit::EncodedVideoSource::Frame) {
    if (!frame_held.exchange(true)) {
      std::this_thread::sleep_for(kQueueOverrunningStall);
    }
  };
  callbacks.on_fell_behind = [&]() {
    fell_behind = true;
    throw 42;
  };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(buildPassthroughPipelineDescription(kOpenH264Source), false, PipelineOutput::EncodedH264);
  const bool overran = waitUntil([&]() { return fell_behind.load(); });
  pipeline.stop();

  EXPECT_TRUE(overran);
}

TEST_F(StreamTest, PipelineContainsNonStandardFrameCallbackException)
{
  std::atomic<bool> failed{false};
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  callbacks.on_frame = [](const livekit::VideoFrame &, std::int64_t) { throw 42; };
  // The sample callback fails the frame, so the pipeline posts an error.
  callbacks.on_failed = [&](const std::string &) { failed = true; };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(buildPipelineDescription("videotestsrc is-live=true ! video/x-raw,width=64,height=48", ""), false);
  const bool reported = waitUntil([&]() { return failed.load(); });
  pipeline.stop();

  EXPECT_TRUE(reported);
}

// Counts overruns of any pipeline's bridge queue through a GLib emission hook on
// the queue's "overrun" signal, independent of the pipeline's own handler.
class BridgeQueueOverrunCounter final
{
public:
  BridgeQueueOverrunCounter()
  {
    const GstObjectPtr<GstElementFactory> queue_factory(gst_element_factory_find("queue"));
    if (queue_factory == nullptr) {
      throw std::runtime_error("The queue element is unavailable.");
    }
    const GstObjectPtr<GstPluginFeature> loaded_feature(
      gst_plugin_feature_load(GST_PLUGIN_FEATURE(queue_factory.get())));
    if (loaded_feature == nullptr) {
      throw std::runtime_error("Failed to load the queue element.");
    }

    // The signal exists once the queue class is initialized; keep it referenced.
    queue_class_ = g_type_class_ref(gst_element_factory_get_element_type(GST_ELEMENT_FACTORY(loaded_feature.get())));
    signal_id_ = g_signal_lookup("overrun", G_TYPE_FROM_CLASS(queue_class_));
    hook_id_ = g_signal_add_emission_hook(signal_id_, 0U, &BridgeQueueOverrunCounter::onOverrun, &count_, nullptr);
  }

  ~BridgeQueueOverrunCounter()
  {
    g_signal_remove_emission_hook(signal_id_, hook_id_);
    g_type_class_unref(queue_class_);
  }

  BridgeQueueOverrunCounter(const BridgeQueueOverrunCounter &) = delete;
  BridgeQueueOverrunCounter & operator=(const BridgeQueueOverrunCounter &) = delete;
  BridgeQueueOverrunCounter(BridgeQueueOverrunCounter &&) = delete;
  BridgeQueueOverrunCounter & operator=(BridgeQueueOverrunCounter &&) = delete;

  std::size_t count() const
  {
    return count_.load();
  }

private:
  static gboolean onOverrun(GSignalInvocationHint *, guint, const GValue * param_values, gpointer count)
  {
    const GCharPtr name(gst_object_get_name(GST_OBJECT(g_value_get_object(&param_values[0]))));
    if (name != nullptr && std::string(name.get()) == kBridgeVideoQueueName) {
      static_cast<std::atomic<std::size_t> *>(count)->fetch_add(1U);
    }
    // Keep the hook installed.
    return TRUE;
  }

  gpointer queue_class_ = nullptr;
  guint signal_id_ = 0U;
  gulong hook_id_ = 0U;
  std::atomic<std::size_t> count_{0U};
};

TEST_F(StreamTest, PassthroughPipelineToleratesMissingFellBehindCallback)
{
  const BridgeQueueOverrunCounter overruns;
  std::atomic<bool> frame_held{false};
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  callbacks.on_encoded_frame = [&](livekit::EncodedVideoSource::Frame) {
    if (!frame_held.exchange(true)) {
      std::this_thread::sleep_for(kQueueOverrunningStall);
    }
  };
  callbacks.on_fell_behind = {};
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(buildPassthroughPipelineDescription(kOpenH264Source), false, PipelineOutput::EncodedH264);
  const bool held = waitUntil([&]() { return frame_held.load(); });
  std::this_thread::sleep_for(kQueueOverrunningStall + std::chrono::milliseconds(250));
  pipeline.stop();

  EXPECT_TRUE(held);
  EXPECT_GT(overruns.count(), 0U);
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
  test_support::ScopedLogCapture logs;
  constexpr char kHint[] = "hint=\"the source may not be H.264; passthrough supports H.264 only\"";
  // A static vp8enc ! h264parse link already fails to parse (and config validation
  // rejects it); the demuxer's dynamic pad fails at runtime like an RTSP camera would.
  StreamSpec spec =
    makePassthroughSpec("videotestsrc is-live=true ! vp8enc deadline=1 ! matroskamux streamable=true ! matroskademux");
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool hint_logged =
    waitUntil([&]() { return !logs.findLine({"video_stream_pipeline_recovery_scheduled", kHint}).empty(); });
  stream.close();

  ASSERT_TRUE(hint_logged);
  const std::string line = logs.findLine({"video_stream_pipeline_recovery_scheduled"});
  EXPECT_NE(line.find("offered_caps=\"video/x-vp8"), std::string::npos) << line;
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 0U);
}

TEST_F(StreamTest, PassthroughStreamSkipsToRequestedKeyFrameAfterFallingBehind)
{
  std::atomic<bool> capture_stalled{false};
  std::promise<void> capture_stalled_promise;
  std::future<void> capture_stalled_future = capture_stalled_promise.get_future();
  std::promise<void> release_capture_promise;
  std::shared_future<void> release_capture_future = release_capture_promise.get_future().share();
  test_support::ScopedLogCapture logs;
  // The publisher logs the first captured frame while it holds its lock, after the
  // publish; blocking there stalls the sample thread while the source keeps producing.
  logs.setLineHook([&](const std::string & line) {
    if (line.find("video_stream_frame_captured") == std::string::npos || capture_stalled.exchange(true)) {
      return;
    }
    capture_stalled_promise.set_value();
    (void)release_capture_future.wait_for(std::chrono::seconds(10));
  });
  StreamSpec spec = makePassthroughSpec(kOpenH264Source);
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool stalled = capture_stalled_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  std::this_thread::sleep_for(kQueueOverrunningStall);
  release_capture_promise.set_value();
  const bool resumed = waitUntil([&]() { return !logs.findLine({"video_stream_passthrough_resumed"}).empty(); });
  stream.close();

  ASSERT_TRUE(stalled);
  EXPECT_FALSE(logs.findLine({"video_stream_passthrough_fell_behind", "stream_key=other_video:test"}).empty());
  ASSERT_TRUE(resumed);
  const std::string resumed_line = logs.findLine({"video_stream_passthrough_resumed"});
  EXPECT_EQ(resumed_line.find("discarded_frames=0"), std::string::npos) << resumed_line;
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 1U);
}

TEST_F(StreamTest, PassthroughStreamSlowFirstPublishIsNotFallingBehind)
{
  bool publish_started = false;
  std::promise<void> publish_started_promise;
  std::future<void> publish_started_future = publish_started_promise.get_future();
  std::promise<void> release_publish_promise;
  std::shared_future<void> release_publish_future = release_publish_promise.get_future().share();
  test_support::ScopedLogCapture logs;
  StreamSpec spec = makePassthroughSpec(kOpenH264Source);
  FakeRoomConnection connection;
  // Publishing on the first keyframe waits on the server while the live source keeps
  // producing; a publish shorter than the bridge queue fits the backlog.
  connection.state->publish_video_track_hook = [&](const std::string &) {
    if (!publish_started) {
      publish_started = true;
      publish_started_promise.set_value();
    }
    (void)release_publish_future.wait_for(std::chrono::seconds(10));
  };
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  const bool publish_blocked = publish_started_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  std::this_thread::sleep_for(kQueueFittingStall);
  release_publish_promise.set_value();
  const bool captured = waitUntil([&]() { return !logs.findLine({"video_stream_frame_captured"}).empty(); });
  // Let the backlog from the publish drain while the source stays live.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  stream.close();

  ASSERT_TRUE(publish_blocked);
  ASSERT_TRUE(captured);
  EXPECT_EQ(logs.findLine({"video_stream_passthrough_fell_behind"}), "");
  EXPECT_EQ(logs.findLine({"video_stream_passthrough_resumed"}), "");
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 1U);
}

TEST_F(StreamTest, PassthroughLogsWhileWaitingForFirstKeyFrame)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, makePassthroughSpec(kOpenH264Source));
  const std::function<void()> request_key_frame = []() {};

  publisher.captureEncoded(makeEncodedFrame(false), request_key_frame);
  publisher.captureEncoded(makeEncodedFrame(false), request_key_frame);
  publisher.captureEncoded(makeEncodedFrame(true), request_key_frame);
  // Later waits follow a logged fall behind or restart and end with the resumed log.
  publisher.awaitKeyFrame();
  publisher.captureEncoded(makeEncodedFrame(false), request_key_frame);

  // Throttled: one line for the two frames discarded before the first keyframe.
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_waiting_for_key_frame"}), 1U);
  EXPECT_FALSE(
    logs
      .findLine({"video_stream_passthrough_waiting_for_key_frame", "stream_key=other_video:test", "discarded_frames=1"})
      .empty());
}

TEST_F(StreamTest, PassthroughStreamPublishFailureKeepsPipelineRunning)
{
  test_support::ScopedLogCapture logs;
  StreamSpec spec = makePassthroughSpec(kOpenH264Source);
  FakeRoomConnection connection;
  std::atomic<int> publish_attempts{0};
  connection.state->publish_video_track_hook = [&](const std::string &) {
    if (++publish_attempts == 1) {
      throw std::runtime_error("simulated video publish failure");
    }
  };
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  stream.start();
  // The source only sends a keyframe when asked, so this one was requested after the failure.
  const bool captured = waitUntil([&]() { return !logs.findLine({"video_stream_frame_captured"}).empty(); });
  stream.close();

  ASSERT_TRUE(captured);
  EXPECT_EQ(publish_attempts.load(), 2);
  EXPECT_FALSE(logs.findLine({"video_stream_capture_failed", "simulated video publish failure"}).empty());
  EXPECT_EQ(logs.findLine({"video_stream_pipeline_recovery_scheduled"}), "");
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
