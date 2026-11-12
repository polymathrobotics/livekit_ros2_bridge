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

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "audio/gstreamer_pipeline.hpp"
#include "audio/gstreamer_stream.hpp"
#include "audio/pipeline_description.hpp"
#include "audio/track_publisher.hpp"
#include "gtest/gtest.h"
#include "livekit/audio_frame.h"
#include "support/controlled_start_element.hpp"
#include "support/detached_task.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "utils/gstreamer_resources.hpp"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::AudioSource; see ScopedLiveKitInit.
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
  spec.stream_key = "other_audio:test";
  spec.track_name = "lkros.audio.other.test";
  spec.input = OtherInput{"test", "", ""};
  return spec;
}

PipelineCallbacks makeNoOpPipelineCallbacks()
{
  return PipelineCallbacks{
    []() { return false; },
    [](const livekit::AudioFrame &) {},
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  };
}

void expectStartErrorContains(GStreamerPipeline & pipeline, const std::string & description, const char * fragment)
{
  try {
    pipeline.start(description);
    FAIL() << "Expected start to throw an error containing '" << fragment << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(fragment), std::string::npos) << "actual error: " << error.what();
  }
}

class AudioStreamTest : public test_support::RclcppTestSuite
{
protected:
  static void SetUpTestSuite()
  {
    utils::ensureGStreamerInitialized();
    RclcppTestSuite::SetUpTestSuite();
  }
};

TEST_F(AudioStreamTest, PipelineStartRejectsNamedNonAppSink)
{
  GStreamerPipeline pipeline(makeNoOpPipelineCallbacks());

  expectStartErrorContains(
    pipeline, "audiotestsrc is-live=true ! fakesink name=bridge_audio_sink", "must be a GstAppSink");
}

TEST_F(AudioStreamTest, PipelineContainsNonStandardCallbackExceptions)
{
  std::mutex error_mutex;
  std::string capture_error;
  std::string failure_reason;
  PipelineCallbacks callbacks = makeNoOpPipelineCallbacks();
  // on_capture_failed also throws, so the exception reaches the new_sample catch;
  // the failed sample makes the pipeline post an error, which on_failed throws on.
  callbacks.on_frame = [](const livekit::AudioFrame &) { throw 42; };
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

  pipeline.start(buildPipelineDescription("audiotestsrc is-live=true", ""));
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

TEST_F(AudioStreamTest, OtherAudioLifecycleIsIdempotent)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "audiotestsrc is-live=true wave=sine", ""};

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
TEST_F(AudioStreamTest, StartWithUnavailableSourceDoesNotWedge)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "filesrc location=/nonexistent/lkros_missing_source.mp3", ""};
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
TEST_F(AudioStreamTest, SourceThatStaysBrokenKeepsBeingRestarted)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::Normal);
  // The mode is process-wide, so put it back for later tests.
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  StreamSpec spec = makeOtherSpec();
  // About 100 ms of audio, then EOS: well after the restart that started it has returned.
  spec.input = OtherInput{
    "test",
    std::string("audiotestsrc is-live=true num-buffers=4 samplesperbuffer=1600 ! audio/x-raw,rate=48000 ! ") +
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
  expectThrottledEvent(logs, "audio_stream_pipeline_recovery_scheduled", started_at);
  expectThrottledEvent(logs, "audio_stream_restart_failed", started_at);
}

// A failure that arrives while a restart's start() runs, without making it throw,
// lands while the restart callback runs. It must still lead to another restart.
TEST_F(AudioStreamTest, FailureDuringRestartLeadsToAnotherRestart)
{
  using test_support::ControlledStartElement;
  ControlledStartElement::registerFactory();
  ControlledStartElement::setMode(ControlledStartElement::Mode::PostErrorOnStart);
  // The mode is process-wide, so put it back for later tests.
  const ScopeExit restore_mode([]() { ControlledStartElement::setMode(ControlledStartElement::Mode::Normal); });
  StreamSpec spec = makeOtherSpec();
  spec.input =
    OtherInput{"test", std::string("audiotestsrc is-live=true ! ") + ControlledStartElement::kFactoryName, ""};
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

TEST_F(AudioStreamTest, RealPipelineProducesMono48kFrames)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool frame_captured = false;
  int sample_rate = 0;
  int channels = 0;

  PipelineCallbacks callbacks{
    []() { return false; },
    [&](const livekit::AudioFrame & frame) {
      std::lock_guard<std::mutex> lock(mutex);
      sample_rate = frame.sampleRate();
      channels = frame.numChannels();
      frame_captured = true;
      condition.notify_all();
    },
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  };
  GStreamerPipeline pipeline(std::move(callbacks));

  pipeline.start(
    "audiotestsrc is-live=true wave=sine ! audioconvert ! audioresample ! "
    "audio/x-raw,format=S16LE,channels=1,rate=48000 ! appsink name=bridge_audio_sink sync=false drop=true "
    "max-buffers=1");

  std::unique_lock<std::mutex> lock(mutex);
  const bool captured = condition.wait_for(lock, std::chrono::seconds(5), [&]() { return frame_captured; });
  lock.unlock();
  pipeline.stop();

  ASSERT_TRUE(captured) << "no audio frame captured from the real pipeline";
  EXPECT_EQ(sample_rate, 48000);
  EXPECT_EQ(channels, 1);
}

}  // namespace

}  // namespace livekit_ros2_bridge::audio
