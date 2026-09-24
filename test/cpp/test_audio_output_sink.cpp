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

#include <gst/app/gstappsrc.h>
#include <gst/base/gstbasesink.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "audio/audio_output_sink.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "utils/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::audio
{
namespace
{

// Tests assert external behavior: which reader claimed the sink, that lifecycle
// paths leave the sink rebindable, and that the restart loop stays rate-bounded
// and owner-gated. The pure timing helper is covered directly; buffer timestamps
// as delivered to GStreamer are a POC-verified property covered by the real-path
// integration, not this suite.

class AudioOutputSinkTest : public test_support::RclcppTestSuite
{
protected:
  static void SetUpTestSuite()
  {
    utils::ensureGStreamerInitialized();
    RclcppTestSuite::SetUpTestSuite();
  }
};

std::vector<std::int16_t> makeSamples(std::size_t count)
{
  return std::vector<std::int16_t>(count, 0);
}

// The fragment names a real sink element so pipeline start succeeds in CI.
constexpr char kTestSinkFragment[] = "fakesink sync=false";

std::vector<bool> collectSinkSync(GstElement * pipeline)
{
  std::vector<bool> sync_by_sink;
  utils::GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline)));
  utils::GValueSlot item;
  while (gst_iterator_next(iterator.get(), item.get()) == GST_ITERATOR_OK) {
    auto * element = GST_ELEMENT(g_value_get_object(item.get()));
    if (GST_IS_BASE_SINK(element)) {
      sync_by_sink.push_back(gst_base_sink_get_sync(GST_BASE_SINK(element)) == TRUE);
    }
    item.reset();
  }
  return sync_by_sink;
}

utils::GstElementPtr parsePlaybackPipeline(const std::string & sink_fragment)
{
  return utils::GstElementPtr(
    gst_parse_launch(buildAudioOutputSinkPipelineDescription(sink_fragment).c_str(), nullptr));
}

TEST_F(AudioOutputSinkTest, FirstReaderClaimsAndBindReportsOwnership)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_TRUE(sink.bind(1, 48000, 1));
  EXPECT_FALSE(sink.bind(2, 48000, 1));

  sink.unbind(1);
  EXPECT_TRUE(sink.bind(2, 48000, 1));
}

TEST_F(AudioOutputSinkTest, FailedInitialStartKeepsTheClaimForRetry)
{
  // An empty fragment makes startPipelineLocked() throw deterministically. The
  // reader must still own the sink afterwards so its live frame cadence keeps
  // re-arming the restart loop and playback recovers when the device returns.
  AudioOutputSink sink("");

  EXPECT_TRUE(sink.bind(1, 48000, 1));
  EXPECT_FALSE(sink.bind(2, 48000, 1));
  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));

  sink.unbind(1);
  EXPECT_TRUE(sink.bind(2, 48000, 1));
}

TEST_F(AudioOutputSinkTest, PushFromNonOwnerIsDroppedWithoutEffect)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_TRUE(sink.bind(1, 48000, 1));

  // A second live output track's frames are dropped, never played.
  EXPECT_NO_THROW(sink.push(2, makeSamples(480).data(), 480));
  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));

  sink.unbind(1);
}

TEST_F(AudioOutputSinkTest, UnbindReleasesClaimForNextTrack)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_TRUE(sink.bind(1, 48000, 1));
  sink.push(1, makeSamples(480).data(), 480);
  sink.unbind(1);

  // Lease-handover rebind: the next output track claims on its first frame.
  EXPECT_TRUE(sink.bind(2, 48000, 1));
  EXPECT_NO_THROW(sink.push(2, makeSamples(480).data(), 480));
}

TEST_F(AudioOutputSinkTest, UnbindFromNonOwnerIsANoOp)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_TRUE(sink.bind(1, 48000, 1));
  sink.unbind(2);
  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));
  EXPECT_FALSE(sink.bind(2, 48000, 1));
}

TEST_F(AudioOutputSinkTest, StopDisablesBindAndKeepsPushSafe)
{
  AudioOutputSink sink(kTestSinkFragment);

  sink.stop();
  EXPECT_FALSE(sink.bind(1, 48000, 1));
  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));
  sink.stop();
}

TEST_F(AudioOutputSinkTest, PushWithoutBindIsDropped)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));
}

TEST_F(AudioOutputSinkTest, SilenceThroughDoesNotDisturbTheClaim)
{
  AudioOutputSink sink(kTestSinkFragment);

  EXPECT_TRUE(sink.bind(1, 48000, 1));
  // Mute arrives as continuous silent frames; the sink stays claimed and the
  // bridge needs no mute reaction.
  for (int frame_index = 0; frame_index < 10; ++frame_index) {
    EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));
  }
  EXPECT_FALSE(sink.bind(2, 48000, 1));
}

TEST_F(AudioOutputSinkTest, UnbindStopsThePipeline)
{
  AudioOutputSink sink(kTestSinkFragment);

  ASSERT_TRUE(sink.bind(1, 48000, 1));
  EXPECT_TRUE(sink.hasActivePipeline());

  // Reader finalize must release the audio.out.sink device, not just the claim.
  sink.unbind(1);
  EXPECT_FALSE(sink.hasActivePipeline());
}

TEST_F(AudioOutputSinkTest, HandoverStartsAFreshPipelineWithNewCaps)
{
  AudioOutputSink sink(kTestSinkFragment);

  ASSERT_TRUE(sink.bind(1, 48000, 1));
  const std::size_t attempts_after_first_bind = sink.pipelineStartAttempts();
  EXPECT_TRUE(sink.hasActivePipeline());
  EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));

  sink.unbind(1);
  EXPECT_FALSE(sink.hasActivePipeline());

  // The next output track claims on its first frame; a fresh pipeline is built
  // with its own rate/channels, and the old pipeline is gone before it starts.
  ASSERT_TRUE(sink.bind(2, 44100, 2));
  EXPECT_EQ(sink.pipelineStartAttempts(), attempts_after_first_bind + 1);
  EXPECT_TRUE(sink.hasActivePipeline());
  EXPECT_NO_THROW(sink.push(2, makeSamples(44100 * 2).data(), 44100 * 2));
}

TEST_F(AudioOutputSinkTest, IdleSinkDoesNotRestartWhilePipelineIsDown)
{
  // Empty fragment makes the initial start throw, so the sink is claimed but has
  // no pipeline. With no frames arriving, the rate-bounded loop must not cycle.
  AudioOutputSink sink("");

  ASSERT_TRUE(sink.bind(1, 48000, 1));
  const std::size_t attempts_after_bind = sink.pipelineStartAttempts();

  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  EXPECT_EQ(sink.pipelineStartAttempts(), attempts_after_bind);
}

TEST_F(AudioOutputSinkTest, LiveFramesReArmBoundedRestarts)
{
  // With a dead device, each live frame re-arms the restart loop (bounded by the
  // 250 ms delay). Over 700 ms that is at least the initial bind plus a retry.
  AudioOutputSink sink("");

  ASSERT_TRUE(sink.bind(1, 48000, 1));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(700);
  while (std::chrono::steady_clock::now() < deadline) {
    EXPECT_NO_THROW(sink.push(1, makeSamples(480).data(), 480));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_GE(sink.pipelineStartAttempts(), 2U);
}

TEST_F(AudioOutputSinkTest, NoRestartAfterUnbind)
{
  AudioOutputSink sink("");

  ASSERT_TRUE(sink.bind(1, 48000, 1));
  for (int frame_index = 0; frame_index < 5; ++frame_index) {
    sink.push(1, makeSamples(480).data(), 480);
  }

  sink.unbind(1);
  const std::size_t attempts_after_unbind = sink.pipelineStartAttempts();

  // Frames from the released reader are ignored, and no pending/queued restart
  // may reopen the device after the claim is gone.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(700);
  while (std::chrono::steady_clock::now() < deadline) {
    sink.push(1, makeSamples(480).data(), 480);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(sink.pipelineStartAttempts(), attempts_after_unbind);
}

TEST_F(AudioOutputSinkTest, SinkSyncIsTurnedOffOnEveryConfiguredSink)
{
  utils::GstElementPtr pipeline =
    parsePlaybackPipeline("tee name=split ! queue ! fakesink sync=true split. ! queue ! fakesink sync=true");
  ASSERT_NE(pipeline, nullptr);
  ASSERT_EQ(collectSinkSync(pipeline.get()), (std::vector<bool>{true, true}));

  disableAudioOutputSinkSync(pipeline.get());

  EXPECT_EQ(collectSinkSync(pipeline.get()), (std::vector<bool>{false, false}));
}

TEST_F(AudioOutputSinkTest, SinkSyncIsTurnedOffOnSinksAddedLater)
{
  // Stands in for autoaudiosink creating its real sink after parsing.
  utils::GstElementPtr pipeline = parsePlaybackPipeline("bin.( name=later queue ! fakesink sync=true )");
  ASSERT_NE(pipeline, nullptr);
  disableAudioOutputSinkSync(pipeline.get());

  utils::GstElementPtr nested_bin(gst_bin_get_by_name(GST_BIN(pipeline.get()), "later"));
  ASSERT_NE(nested_bin, nullptr);
  GstElement * added_sink = gst_element_factory_make("fakesink", nullptr);
  ASSERT_NE(added_sink, nullptr);
  g_object_set(added_sink, "sync", TRUE, nullptr);
  ASSERT_TRUE(gst_bin_add(GST_BIN(nested_bin.get()), added_sink));

  EXPECT_FALSE(gst_base_sink_get_sync(GST_BASE_SINK(added_sink)));
  EXPECT_EQ(collectSinkSync(pipeline.get()), (std::vector<bool>{false, false}));
}

TEST_F(AudioOutputSinkTest, BacklogBehindASlowOutputStaysWithinTheCap)
{
  // Drains at half real time, so audio pushed faster piles up in front of it.
  utils::GstElementPtr pipeline = parsePlaybackPipeline("identity sleep-time=20000 ! fakesink sync=false");
  ASSERT_NE(pipeline, nullptr);
  utils::GstElementPtr appsrc_element(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeAppSrcName));
  ASSERT_NE(appsrc_element, nullptr);
  utils::GstCapsPtr caps(gst_caps_from_string("audio/x-raw,format=S16LE,layout=interleaved,rate=48000,channels=1"));
  gst_app_src_set_caps(GST_APP_SRC(appsrc_element.get()), caps.get());
  ASSERT_NE(gst_element_set_state(pipeline.get(), GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);

  constexpr std::size_t kSamplesPerFrame = 480;
  constexpr int kFrameCount = 300;  // 3 s of 10 ms frames
  GstClockTime next_pts = 0;
  for (int frame = 0; frame < kFrameCount; ++frame) {
    const AudioOutputBufferTiming timing = computeAudioOutputBufferTiming(kSamplesPerFrame, 1, 48000, next_pts);
    next_pts += timing.duration;
    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, kSamplesPerFrame * sizeof(std::int16_t), nullptr);
    gst_buffer_memset(buffer, 0, 0, kSamplesPerFrame * sizeof(std::int16_t));
    GST_BUFFER_PTS(buffer) = timing.pts;
    GST_BUFFER_DURATION(buffer) = timing.duration;
    ASSERT_EQ(gst_app_src_push_buffer(GST_APP_SRC(appsrc_element.get()), buffer), GST_FLOW_OK);
  }

  guint64 queued_time = 0;
  g_object_get(appsrc_element.get(), "current-level-time", &queued_time, nullptr);
  EXPECT_LE(queued_time, kAudioOutputMaxBacklog + 10 * GST_MSECOND);

  gst_element_set_state(pipeline.get(), GST_STATE_NULL);
}

TEST_F(AudioOutputSinkTest, TimingHelperIsCorrectForMonoAndStereo)
{
  // 480 frames at 48 kHz is 10 ms regardless of channel count; the interleaved
  // sample count only scales the byte size, not the duration.
  const AudioOutputBufferTiming mono = computeAudioOutputBufferTiming(480, 1, 48000, 0);
  EXPECT_EQ(mono.pts, 0U);
  EXPECT_EQ(mono.duration, 10'000'000U);

  const AudioOutputBufferTiming stereo = computeAudioOutputBufferTiming(960, 2, 48000, 0);
  EXPECT_EQ(stereo.duration, 10'000'000U);

  // A running PTS advances by exactly the previous buffer's duration.
  const AudioOutputBufferTiming second = computeAudioOutputBufferTiming(480, 1, 48000, mono.pts + mono.duration);
  EXPECT_EQ(second.pts, 10'000'000U);
}

}  // namespace
}  // namespace livekit_ros2_bridge::audio
