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

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <future>
#include <initializer_list>
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
#include "rcutils/logging.h"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
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

  // Returns how many captured lines contain the fragment.
  std::size_t countLines(const char * fragment) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::size_t count = 0U;
    for (const auto & line : lines()) {
      if (line.find(fragment) != std::string::npos) {
        ++count;
      }
    }
    return count;
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

// A file under the test temp dir, named per process so parallel runs do not
// collide, and removed when the test ends (also when it fails early).
class ScopedTempFile final
{
public:
  explicit ScopedTempFile(const std::string & name)
  : path_(::testing::TempDir() + "lkros_" + std::to_string(::getpid()) + "_" + name)
  {}

  ~ScopedTempFile()
  {
    std::remove(path_.c_str());
  }

  ScopedTempFile(const ScopedTempFile &) = delete;
  ScopedTempFile & operator=(const ScopedTempFile &) = delete;
  ScopedTempFile(ScopedTempFile &&) = delete;
  ScopedTempFile & operator=(ScopedTempFile &&) = delete;

  const std::string & path() const
  {
    return path_;
  }

private:
  std::string path_;
};

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
  ScopedTempFile source_file("audio_restart_source.raw");
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

  ScopedLogCapture logs;
  FakeRoomConnection connection;
  TrackPublisher publisher(connection, spec);
  GStreamerStream stream(spec, publisher);
  stream.start();
  ASSERT_TRUE(waitUntil([&]() { return logs.countLines("audio_stream_pipeline_recovery_scheduled") > 0U; }));

  // From here every restart fails at once; give it several attempts.
  std::remove(source_file.path().c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  stream.close();

  EXPECT_EQ(logs.countLines("audio_stream_pipeline_recovery_scheduled"), 1U);
  EXPECT_EQ(logs.countLines("audio_stream_restart_failed"), 1U);
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
