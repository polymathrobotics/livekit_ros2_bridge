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
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "audio/track_publisher.hpp"
#include "gtest/gtest.h"
#include "livekit/audio_frame.h"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"

namespace livekit_ros2_bridge::audio
{
namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::AudioSource; see ScopedLiveKitInit.
const test_support::ScopedLiveKitInit kLiveKitInit;

StreamSpec makeSpec(const std::string & stream_key, const std::string & track_name)
{
  StreamSpec spec;
  spec.stream_key = stream_key;
  spec.track_name = track_name;
  return spec;
}

livekit::AudioFrame makeFrame(int sample_rate = 48000, int channels = 1, int samples_per_channel = 480)
{
  return livekit::AudioFrame::create(sample_rate, channels, samples_per_channel);
}

TEST(AudioTrackPublisherTest, FirstFramePublishesAndDestructionUnpublishes)
{
  FakeRoomConnection connection;
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("other_audio:lifecycle", "lkros.audio.other.lifecycle"));

  publisher->capture(makeFrame());
  publisher->capture(makeFrame());
  publisher.reset();

  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_audio_track:lkros.audio.other.lifecycle",
      "unpublish_audio_track:lkros.audio.other.lifecycle",
    }));
}

TEST(AudioTrackPublisherTest, PublishOptionsFlowThroughToRoomConnection)
{
  FakeRoomConnection connection;
  StreamSpec spec = makeSpec("other_audio:options", "lkros.audio.other.options");
  livekit::TrackPublishOptions options;
  options.dtx = true;
  options.red = true;
  livekit::AudioEncodingOptions encoding;
  encoding.max_bitrate = 96000U;
  options.audio_encoding = encoding;
  spec.publish_options = options;

  auto publisher = std::make_unique<TrackPublisher>(connection, std::move(spec));

  publisher->capture(makeFrame());

  ASSERT_EQ(connection.state->published_audio_options.size(), 1U);
  const auto & published = connection.state->published_audio_options.front();
  EXPECT_EQ(published.dtx, true);
  EXPECT_EQ(published.red, true);
  ASSERT_TRUE(published.audio_encoding.has_value());
  EXPECT_EQ(published.audio_encoding->max_bitrate, 96000U);
}

TEST(AudioTrackPublisherTest, DestructionUsesBestEffortPublishedTrackCleanup)
{
  FakeRoomConnection connection;
  connection.state->throw_on_unpublish_audio = true;
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makeSpec("other_audio:unpublish_failure", "lkros.audio.other.unpublish_failure"));

  publisher->capture(makeFrame());
  EXPECT_NO_THROW(publisher.reset());

  EXPECT_EQ(
    connection.state->published_audio_track_names, (std::vector<std::string>{"lkros.audio.other.unpublish_failure"}));
  EXPECT_EQ(
    connection.state->unpublished_audio_track_names, (std::vector<std::string>{"lkros.audio.other.unpublish_failure"}));
}

TEST(AudioTrackPublisherTest, TeardownSurvivesForeignAndFailingTrackUnpublish)
{
  FakeRoomConnection connection;

  // A foreign/untracked LocalAudioTrack handle (e.g. a stale SID after a
  // reconnect swapped publications) is treated as unknown and must not throw.
  auto foreign_track = connection.makeSyntheticAudioTrack();
  EXPECT_NO_THROW(connection.unpublishAudioTrack(foreign_track));
  EXPECT_EQ(connection.state->unpublished_audio_track_names, (std::vector<std::string>{kUnknownDataTrackName}));

  // The best-effort teardown path also swallows a throwing unpublish rather
  // than letting a stale-SID failure escape destruction.
  connection.state->throw_on_unpublish_audio = true;
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("other_audio:stale_sid", "lkros.audio.other.stale_sid"));

  publisher->capture(makeFrame());
  EXPECT_NO_THROW(publisher.reset());
}

TEST(AudioTrackPublisherTest, PublishFailureIsNonFatalAndRetriesAfterBackoff)
{
  FakeRoomConnection connection;
  int publish_attempts = 0;
  connection.state->publish_audio_track_hook = [&](const std::string &) {
    ++publish_attempts;
    if (publish_attempts == 1) {
      throw std::runtime_error("simulated audio publish failure");
    }
  };
  // The publish-retry interval is injected at 50 ms so the retry can be
  // observed deterministically: a 60 ms sleep (adding ~60 ms to the suite,
  // well under the old 350 ms default wait) is always past the backoff, and
  // the former in-backoff negative assertion is omitted because with a tiny
  // interval an "immediate" second capture could already land outside it.
  auto publisher = std::make_unique<TrackPublisher>(
    connection,
    makeSpec("other_audio:publish_retry", "lkros.audio.other.publish_retry"),
    std::chrono::milliseconds(50));

  // First frame: the LiveKit publish fails. This must NOT throw or tear the
  // pipeline down; the frame is dropped and a retry is scheduled.
  EXPECT_NO_THROW(publisher->capture(makeFrame()));
  EXPECT_EQ(publish_attempts, 1);

  // After the backoff elapses, the next frame retries the publish and succeeds.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  publisher->capture(makeFrame());
  EXPECT_EQ(publish_attempts, 2);

  publisher.reset();

  // The fake records both publish calls (the failed first one and the retry);
  // only the successful track is unpublished on destruction.
  EXPECT_EQ(
    connection.state->published_audio_track_names,
    (std::vector<std::string>{
      "lkros.audio.other.publish_retry",
      "lkros.audio.other.publish_retry",
    }));
  EXPECT_EQ(
    connection.state->unpublished_audio_track_names, (std::vector<std::string>{"lkros.audio.other.publish_retry"}));
}

}  // namespace
}  // namespace livekit_ros2_bridge::audio
