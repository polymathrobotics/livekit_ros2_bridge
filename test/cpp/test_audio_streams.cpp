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
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "audio/gstreamer_pipeline.hpp"
#include "audio/gstreamer_stream.hpp"
#include "audio/track_publisher.hpp"
#include "gtest/gtest.h"
#include "livekit/audio_frame.h"
#include "support/error_on_start_element.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "support/scoped_temp_file.hpp"
#include "utils/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::AudioSource; see ScopedLiveKitInit.
const test_support::ScopedLiveKitInit kLiveKitInit;

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

// A source that fails while the state change runs (missing file) is delivered
// through the sync bus handler on the calling thread. The failure path must not
// take the stream mutex_: start() has to return or throw promptly, and close()
// has to complete afterwards.
TEST_F(AudioStreamTest, StartWithUnavailableSourceDoesNotWedge)
{
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{"test", "filesrc location=/nonexistent/lkros_missing_source.mp3", ""};

  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);

  auto started = std::async(std::launch::async, [&stream]() {
    try {
      stream.start();
    } catch (const std::exception &) {
    }
  });
  EXPECT_EQ(started.wait_for(std::chrono::seconds(10)), std::future_status::ready);

  auto closed = std::async(std::launch::async, [&stream]() { stream.close(); });
  EXPECT_EQ(closed.wait_for(std::chrono::seconds(10)), std::future_status::ready);
}

// A source that keeps failing is retried every restart delay; its warnings are
// throttled instead of logged on every attempt.
TEST_F(AudioStreamTest, RepeatedRestartFailuresAreLoggedOnce)
{
  // 10 ms of mono S16LE silence: the source plays it, reaches EOS, and restarts.
  const test_support::ScopedTempFile source_file("lkros_audio_restart_source_");
  {
    std::FILE * file = std::fopen(source_file.path().c_str(), "wb");
    ASSERT_NE(file, nullptr);
    const std::vector<std::int16_t> samples(480U, 0);
    std::fwrite(samples.data(), sizeof(std::int16_t), samples.size(), file);
    std::fclose(file);
  }
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{
    "test",
    "filesrc location=" + source_file.path() +
      " ! rawaudioparse format=pcm pcm-format=s16le sample-rate=48000 num-channels=1",
    ""};

  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);
  stream.start();
  ASSERT_TRUE(waitUntil([&]() { return logs.countLines({"audio_stream_pipeline_recovery_scheduled"}) > 0U; }));

  // From here every restart fails at once; give it several attempts.
  source_file.remove();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  stream.close();

  EXPECT_EQ(logs.countLines({"audio_stream_pipeline_recovery_scheduled"}), 1U);
  EXPECT_EQ(logs.countLines({"audio_stream_restart_failed"}), 1U);
}

// A failure that arrives while a restart's start() runs, without making it throw,
// lands while the restart callback runs. It must still lead to another restart.
TEST_F(AudioStreamTest, FailureDuringRestartLeadsToAnotherRestart)
{
  test_support::ErrorOnStartElement::registerFactory();
  StreamSpec spec = makeOtherSpec();
  spec.input = OtherInput{
    "test", std::string("audiotestsrc is-live=true ! ") + test_support::ErrorOnStartElement::kFactoryName, ""};
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
