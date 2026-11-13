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

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
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

StreamSpec makePassthroughSpec(const std::string & source_fragment)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", source_fragment, "", true};
  spec.publish_options.video_encoder = livekit::VideoEncoderBackend::PreEncoded;
  spec.publish_options.simulcast = false;
  return spec;
}

EncodedVideoFrame makeEncodedFrame(bool is_keyframe)
{
  EncodedVideoFrame encoded;
  encoded.codec = livekit::VideoCodec::H264;
  encoded.frame.is_keyframe = is_keyframe;
  encoded.frame.width = 64U;
  encoded.frame.height = 48U;
  encoded.frame.timestamp_us = 1000;
  // Annex-B start code and an IDR (type 5) or non-IDR (type 1) slice header.
  const std::uint8_t nal_header = is_keyframe ? 0x65U : 0x41U;
  encoded.frame.data = {0x00U, 0x00U, 0x00U, 0x01U, nal_header, 0x88U};
  return encoded;
}

// Codec, keyframe, and whether the keyframe lacked parameter sets.
using ForwardedFrame = std::tuple<livekit::VideoCodec, bool, bool>;

struct PushedBuffer
{
  std::vector<std::uint8_t> bytes;
  bool delta_unit = false;
};

// Pushes the buffers through a passthrough pipeline whose appsrc offers `caps`, as a parser would.
std::vector<ForwardedFrame> forwardThroughPassthroughPipeline(
  const std::string & caps, const std::vector<PushedBuffer> & buffers)
{
  std::mutex forwarded_mutex;
  std::vector<ForwardedFrame> forwarded;
  // Both callbacks run on the streaming thread, the warning just before its frame.
  bool lacked_parameter_sets = false;
  PipelineCallbacks callbacks = test_support::makeNoOpVideoPipelineCallbacks();
  callbacks.on_key_frame_without_parameter_sets = [&](livekit::VideoCodec) { lacked_parameter_sets = true; };
  callbacks.on_encoded_frame = [&](EncodedVideoFrame encoded) {
    const std::lock_guard<std::mutex> lock(forwarded_mutex);
    forwarded.emplace_back(encoded.codec, encoded.frame.is_keyframe, std::exchange(lacked_parameter_sets, false));
  };
  GStreamerPipeline pipeline(callbacks);

  pipeline.start(
    buildPassthroughPipelineDescription("appsrc name=" + std::string(kBridgeAppSrcName) + " caps=\"" + caps + "\""),
    true,
    PipelineOutput::EncodedVideo);
  for (const PushedBuffer & pushed : buffers) {
    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, pushed.bytes.size(), nullptr);
    (void)gst_buffer_fill(buffer, 0U, pushed.bytes.data(), pushed.bytes.size());
    if (pushed.delta_unit) {
      GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
    }
    (void)gst_app_src_push_buffer(pipeline.appsrc(), buffer);
  }
  (void)test_support::waitUntil(
    [&]() {
      const std::lock_guard<std::mutex> lock(forwarded_mutex);
      return forwarded.size() >= buffers.size();
    },
    std::chrono::seconds(10));
  pipeline.stop();
  return forwarded;
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
TEST_F(StreamTest, PassthroughPipelineReportsUnsupportedSourceAsCapsMismatch)
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
    PipelineOutput::EncodedVideo);
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
  callbacks.on_encoded_frame = [&](EncodedVideoFrame) {
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
    PipelineOutput::EncodedVideo);
  const bool reported = test_support::waitUntil(reported_fell_behind, std::chrono::seconds(10));
  pipeline.stop();

  EXPECT_TRUE(reported);
  EXPECT_GE(
    logs.countLines(
      {"video_pipeline_callback_failed", "callback=queue_overrun", "error=simulated fell behind failure"}),
    1U);
}

// The bytes stand in for what h264parse, h265parse, and av1parse emit with these caps.
TEST_F(StreamTest, PassthroughPipelineTagsFramesWithTheCodecOfTheirCaps)
{
  using livekit::VideoCodec;
  EXPECT_EQ(
    forwardThroughPassthroughPipeline(
      "video/x-h264,stream-format=byte-stream,alignment=au,width=64,height=48",
      {{{0x00U, 0x00U, 0x00U, 0x01U, 0x65U, 0x88U}}}),
    (std::vector<ForwardedFrame>{{VideoCodec::H264, true, true}}));

  // An IDR without VPS, SPS and PPS, then a CRA: h265parse leaves DELTA_UNIT clear on both.
  EXPECT_EQ(
    forwardThroughPassthroughPipeline(
      "video/x-h265,stream-format=byte-stream,alignment=au,width=64,height=48",
      {{{0x00U, 0x00U, 0x00U, 0x01U, 0x26U, 0x01U, 0xAFU}}, {{0x00U, 0x00U, 0x00U, 0x01U, 0x2AU, 0x01U, 0xAFU}}}),
    (std::vector<ForwardedFrame>{{VideoCodec::H265, true, true}, {VideoCodec::H265, false, false}}));

  // av1parse clears DELTA_UNIT only on keyframes. Temporal delimiter, sequence header, and frame OBUs.
  const std::vector<std::uint8_t> key_frame_with_sequence_header{
    0x12U, 0x00U, 0x0AU, 0x01U, 0x00U, 0x32U, 0x01U, 0x10U};
  const std::vector<std::uint8_t> frame_alone{0x12U, 0x00U, 0x32U, 0x01U, 0x10U};
  EXPECT_EQ(
    forwardThroughPassthroughPipeline(
      "video/x-av1,stream-format=obu-stream,alignment=tu,width=64,height=48",
      {{key_frame_with_sequence_header}, {frame_alone, true}, {frame_alone}}),
    (std::vector<ForwardedFrame>{
      {VideoCodec::AV1, true, false}, {VideoCodec::AV1, false, false}, {VideoCodec::AV1, true, true}}));
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

TEST_F(StreamTest, PassthroughStreamExplainsUnsupportedSource)
{
  test_support::ScopedLogCapture logs;
  // The demuxer's dynamic pad fails at runtime, like an RTSP camera that sends VP8 would.
  const auto harness = std::make_shared<OtherStreamHarness>(
    makePassthroughSpec("videotestsrc is-live=true ! vp8enc deadline=1 ! matroskamux streamable=true ! matroskademux"));
  const auto start_stream = [harness]() { harness->stream.start(); };
  const auto close_stream = [harness]() { harness->stream.close(); };
  const auto explained = [&]() {
    return logs.countLines(
             {"video_stream_pipeline_recovery_scheduled",
              "hint=\"the source must output H.264 or H.265 as an Annex-B byte stream with one access unit per "
              "buffer, or AV1 as an OBU stream with one temporal unit per buffer; end it with h264parse "
              "config-interval=-1, h265parse config-interval=-1, or av1parse\"",
              "offered_caps=\"video/x-vp8"}) > 0U;
  };

  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
  const bool hint_logged = test_support::waitUntil(explained, std::chrono::seconds(10));
  EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";

  EXPECT_TRUE(hint_logged);
  EXPECT_EQ(harness->connection.state->publishedVideoTrackCount(), 0U);
}

// A viewer that starts at a keyframe without its parameter sets cannot decode it. For each codec,
// the source sends three such keyframes and ends, and is restarted; the stream warns once, with
// the hint for that codec.
TEST_F(StreamTest, PassthroughStreamWarnsOnceAboutKeyFramesWithoutParameterSets)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::Normal);

  struct KeyFrameCase
  {
    const char * codec;
    const char * caps;
    std::string key_frame;
    const char * hint;
  };

  // As a source without h264parse or h265parse config-interval=-1, or an AV1 encoder that sends
  // its sequence header only once, can send them.
  const std::vector<KeyFrameCase> key_frame_cases{
    {"h264",
     "video/x-h264,stream-format=byte-stream,alignment=au",
     std::string("\x00\x00\x00\x01\x65\x88\x00", 7U),
     "a keyframe arrived without SPS and PPS, so viewers who join late or reconnect may not be able to decode "
     "the video; end the source with h264parse config-interval=-1"},
    {"h265",
     "video/x-h265,stream-format=byte-stream,alignment=au",
     std::string("\x00\x00\x00\x01\x26\x01\xAF", 7U),
     "a keyframe arrived without VPS, SPS and PPS, so viewers who join late or reconnect may not be able to "
     "decode the video; end the source with h265parse config-interval=-1"},
    {"av1",
     "video/x-av1,stream-format=obu-stream,alignment=tu",
     std::string("\x12\x00\x32\x01\x10", 5U),
     "a keyframe arrived without a sequence header, so viewers who join late or reconnect may not be able to "
     "decode the video; set the encoder to send a sequence header with every keyframe"},
  };

  for (const KeyFrameCase & key_frame_case : key_frame_cases) {
    SCOPED_TRACE(key_frame_case.codec);
    const std::filesystem::path key_frame_path =
      std::filesystem::temp_directory_path() / "lkros_key_frame_without_parameter_sets";
    {
      std::ofstream file(key_frame_path, std::ios::binary);
      file.write(key_frame_case.key_frame.data(), static_cast<std::streamsize>(key_frame_case.key_frame.size()));
    }
    const ScopeExit remove_key_frame([&]() { std::filesystem::remove(key_frame_path); });
    test_support::ScopedLogCapture logs;
    // multifilesrc sends the whole file as each buffer.
    const auto harness = std::make_shared<OtherStreamHarness>(makePassthroughSpec(
      "multifilesrc location=" + key_frame_path.string() + " num-buffers=3 caps=\"" + key_frame_case.caps +
      ",width=64,height=48\" ! " + ControlledStartElement::kFactoryName));
    const auto attempts_before = ControlledStartElement::startAttempts();
    // The third start follows the end of the second run, so its frames have arrived too.
    const auto second_run_ended = [attempts_before]() {
      return ControlledStartElement::startAttempts() - attempts_before >= 3U;
    };
    const auto start_stream = [harness]() { harness->stream.start(); };
    const auto close_stream = [harness]() { harness->stream.close(); };

    ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
    const bool restarted = test_support::waitUntil(second_run_ended, std::chrono::seconds(10));
    EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";

    ASSERT_TRUE(restarted);
    EXPECT_EQ(logs.countLines({"video_stream_passthrough_missing_parameter_sets", "stream_key=other_video:test"}), 1U);
    const std::string codec_field = std::string("codec=") + key_frame_case.codec;
    const std::string hint_field = std::string("hint=\"") + key_frame_case.hint + "\"";
    EXPECT_EQ(
      logs.countLines({"video_stream_passthrough_missing_parameter_sets", codec_field.c_str(), hint_field.c_str()}),
      1U);
  }
}

// A restarted source can start mid-stream, so after each restart the publisher
// discards frames, asking for a keyframe, until one arrives.
TEST_F(StreamTest, PassthroughStreamWaitsForKeyFrameAfterRestart)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::Normal);
  // A source that sends nothing and ends at once, so the stream keeps restarting
  // and every frame comes from the test.
  const auto harness = std::make_shared<OtherStreamHarness>(
    makePassthroughSpec(std::string("fakesrc num-buffers=0 ! ") + ControlledStartElement::kFactoryName));
  const auto attempts_before = ControlledStartElement::startAttempts();
  const auto attempts = [attempts_before]() { return ControlledStartElement::startAttempts() - attempts_before; };
  const auto key_frame_requests = std::make_shared<std::atomic<int>>(0);
  const std::function<void()> request_key_frame = [key_frame_requests]() { ++*key_frame_requests; };
  const auto start_stream = [harness]() { harness->stream.start(); };
  const auto close_stream = [harness]() { harness->stream.close(); };

  ASSERT_TRUE(test_support::finishesWithin(kHangTimeout, start_stream)) << "start() did not return.";
  harness->track_publisher.captureEncoded(makeEncodedFrame(true), request_key_frame);
  // A restart that had already begun may have waited before the keyframe; the second did not.
  const std::size_t attempts_after_key_frame = attempts();
  const bool restarted =
    test_support::waitUntil([&]() { return attempts() >= attempts_after_key_frame + 2U; }, std::chrono::seconds(5));
  harness->track_publisher.captureEncoded(makeEncodedFrame(false), request_key_frame);
  EXPECT_TRUE(test_support::finishesWithin(kHangTimeout, close_stream)) << "close() did not return.";

  ASSERT_TRUE(restarted) << "start attempts: " << attempts();
  EXPECT_EQ(key_frame_requests->load(), 1);
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
