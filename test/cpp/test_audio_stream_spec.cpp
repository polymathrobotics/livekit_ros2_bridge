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

#include <stdexcept>
#include <string>
#include <utility>

#include "audio/stream_spec.hpp"
#include "gtest/gtest.h"

namespace livekit_ros2_bridge::audio
{
namespace
{

void expectThrowsWithMessage(std::function<void()> callable, const char * expected_message)
{
  try {
    callable();
    FAIL() << "Expected exception";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), expected_message);
  }
}

TEST(AudioStreamSpecTest, ResolveOtherSourceSpecTrimsOtherSourceName)
{
  StreamConfig config = makeDefaultConfig();
  livekit::TrackPublishOptions expected_options;
  expected_options.dtx = true;
  expected_options.red = false;
  livekit::AudioEncodingOptions encoding;
  encoding.max_bitrate = 64000U;
  expected_options.audio_encoding = encoding;

  OtherSource source;
  source.source_fragment = "audiotestsrc is-live=true wave=sine";
  source.transform_fragment = "volume volume=0.5";
  source.publish_options = expected_options;

  config.other_sources.emplace("cab_mic", std::move(source));

  const auto spec = resolveOtherSourceSpec(config, "  cab_mic  ");
  const auto & input = spec.input;

  EXPECT_EQ(spec.stream_key, "other_audio:cab_mic");
  EXPECT_EQ(spec.track_name, "lkros.audio.other.cab_mic");
  EXPECT_EQ(input.name, "cab_mic");
  EXPECT_EQ(input.source_fragment, "audiotestsrc is-live=true wave=sine");
  EXPECT_EQ(input.transform_fragment, "volume volume=0.5");
  EXPECT_EQ(spec.publish_options.dtx, expected_options.dtx);
  EXPECT_EQ(spec.publish_options.red, expected_options.red);
  ASSERT_TRUE(spec.publish_options.audio_encoding.has_value());
  EXPECT_EQ(spec.publish_options.audio_encoding->max_bitrate, 64000U);
}

TEST(AudioStreamSpecTest, ResolveOtherSourceSpecPercentEncodesTrackNameSuffix)
{
  StreamConfig config = makeDefaultConfig();

  OtherSource source;
  source.source_fragment = "audiotestsrc is-live=true wave=sine";

  config.other_sources.emplace("/sources/cab:mic%", std::move(source));

  const auto spec = resolveOtherSourceSpec(config, "/sources/cab:mic%");

  EXPECT_EQ(spec.track_name, "lkros.audio.other.%2Fsources%2Fcab%3Amic%25");
}

TEST(AudioStreamSpecTest, ResolveOtherSourceSpecRejectsInvalidNames)
{
  const StreamConfig config = makeDefaultConfig();

  expectThrowsWithMessage(
    [&]() { (void)resolveOtherSourceSpec(config, "sources/missing"); },
    "Unknown other audio source 'sources/missing'.");
  expectThrowsWithMessage([&]() { (void)resolveOtherSourceSpec(config, " \t\n "); }, "Invalid other audio name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge::audio
