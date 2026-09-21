// Copyright (c) 2025-present Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
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

TEST(AudioStreamSpecTest, ResolveExternalSourceSpecTrimsExternalSourceName)
{
  StreamConfig config = makeDefaultConfig();
  livekit::TrackPublishOptions expected_options;
  expected_options.dtx = true;
  expected_options.red = false;
  livekit::AudioEncodingOptions encoding;
  encoding.max_bitrate = 64000U;
  expected_options.audio_encoding = encoding;

  ExternalSource source;
  source.source_fragment = "audiotestsrc is-live=true wave=sine";
  source.transform_fragment = "volume volume=0.5";
  source.publish_options = expected_options;

  config.external_sources.emplace("cab_mic", std::move(source));

  const auto spec = resolveExternalSourceSpec(config, "  cab_mic  ");
  const auto & input = spec.input;

  EXPECT_EQ(spec.stream_key, "external_audio:cab_mic");
  EXPECT_EQ(spec.track_name, "lkros.audio.external.cab_mic");
  EXPECT_EQ(input.name, "cab_mic");
  EXPECT_EQ(input.source_fragment, "audiotestsrc is-live=true wave=sine");
  EXPECT_EQ(input.transform_fragment, "volume volume=0.5");
  EXPECT_EQ(spec.publish_options.dtx, expected_options.dtx);
  EXPECT_EQ(spec.publish_options.red, expected_options.red);
  ASSERT_TRUE(spec.publish_options.audio_encoding.has_value());
  EXPECT_EQ(spec.publish_options.audio_encoding->max_bitrate, 64000U);
}

TEST(AudioStreamSpecTest, ResolveExternalSourceSpecPercentEncodesTrackNameSuffix)
{
  StreamConfig config = makeDefaultConfig();

  ExternalSource source;
  source.source_fragment = "audiotestsrc is-live=true wave=sine";

  config.external_sources.emplace("/sources/cab:mic%", std::move(source));

  const auto spec = resolveExternalSourceSpec(config, "/sources/cab:mic%");

  EXPECT_EQ(spec.track_name, "lkros.audio.external.%2Fsources%2Fcab%3Amic%25");
}

TEST(AudioStreamSpecTest, ResolveExternalSourceSpecRejectsInvalidNames)
{
  const StreamConfig config = makeDefaultConfig();

  expectThrowsWithMessage(
    [&]() { (void)resolveExternalSourceSpec(config, "sources/missing"); },
    "Unknown external audio source 'sources/missing'.");
  expectThrowsWithMessage([&]() { (void)resolveExternalSourceSpec(config, " \t\n "); }, "Invalid external audio name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge::audio
