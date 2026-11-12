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

#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "audio/audio_output_sink.hpp"
#include "audio/gstreamer_pipeline.hpp"
#include "audio/pipeline_description.hpp"
#include "gtest/gtest.h"
#include "livekit/audio_frame.h"
#include "livekit/video_frame.h"
#include "support/ros_test_support.hpp"
#include "utils/gstreamer_resources.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/pipeline_description.hpp"

// CMake runs this binary with GST_TRACERS=leaks. The test starts and stops each
// kind of pipeline, then deinitializes GStreamer so the leaks tracer reports any
// GStreamer object still alive, and fails if it does.

namespace livekit_ros2_bridge
{
namespace
{

constexpr auto kFrameTimeout = std::chrono::seconds(10);
constexpr int kFramesBeforeStop = 3;

bool isLeaksTracerActive()
{
  GList * tracers = gst_tracing_get_active_tracers();
  bool found = false;
  for (GList * entry = tracers; entry != nullptr; entry = entry->next) {
    found = found || std::string(G_OBJECT_TYPE_NAME(entry->data)) == "GstLeaksTracer";
  }
  g_list_free_full(tracers, gst_object_unref);
  return found;
}

void recordWarning(const gchar * log_domain, GLogLevelFlags log_level, const gchar * message, gpointer user_data)
{
  if ((log_level & (G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL | G_LOG_LEVEL_WARNING)) != 0) {
    static_cast<std::vector<std::string> *>(user_data)->push_back(message != nullptr ? message : "");
  }
  g_log_default_handler(log_domain, log_level, message, nullptr);
}

// The leaks tracer reports at deinit and only warns, so the warnings are
// collected. GStreamer cannot be used again in this process afterwards.
std::vector<std::string> deinitGStreamerCollectingWarnings()
{
  std::vector<std::string> warnings;
  const GLogFunc previous_handler = g_log_set_default_handler(recordWarning, &warnings);
  gst_deinit();
  g_log_set_default_handler(previous_handler, nullptr);
  return warnings;
}

bool runVideoPipeline()
{
  std::atomic<int> frame_count{0};
  video::GStreamerPipeline pipeline(video::PipelineCallbacks{
    []() { return false; },
    [&](const livekit::VideoFrame &, std::int64_t) { ++frame_count; },
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  });

  pipeline.start(
    video::buildPipelineDescription("videotestsrc is-live=true ! video/x-raw,width=64,height=48", ""), false);
  const bool received = test_support::waitUntil([&]() { return frame_count >= kFramesBeforeStop; }, kFrameTimeout);
  pipeline.stop();
  return received;
}

bool runAudioPipeline()
{
  std::atomic<int> frame_count{0};
  audio::GStreamerPipeline pipeline(audio::PipelineCallbacks{
    []() { return false; },
    [&](const livekit::AudioFrame &) { ++frame_count; },
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  });

  pipeline.start(audio::buildPipelineDescription("audiotestsrc is-live=true", ""));
  const bool received = test_support::waitUntil([&]() { return frame_count >= kFramesBeforeStop; }, kFrameTimeout);
  pipeline.stop();
  return received;
}

bool runAudioOutputSink()
{
  const std::vector<std::int16_t> samples(480, 0);
  audio::AudioOutputSink sink("fakesink sync=false");

  const bool bound = sink.bind(1, 48000, 1);
  const bool started = sink.hasActivePipeline();
  for (int frame = 0; frame < kFramesBeforeStop; ++frame) {
    sink.push(1, samples.data(), samples.size());
  }
  sink.unbind(1);
  return bound && started && !sink.hasActivePipeline();
}

TEST(GStreamerLeaksTest, StoppedPipelinesLeaveNoGStreamerObjectsAlive)
{
  utils::ensureGStreamerInitialized();
  ASSERT_TRUE(isLeaksTracerActive()) << "Run this binary with GST_TRACERS=leaks, as its CMake test does.";

  EXPECT_TRUE(runVideoPipeline()) << "The video pipeline delivered no frames.";
  EXPECT_TRUE(runAudioPipeline()) << "The audio pipeline delivered no frames.";
  EXPECT_TRUE(runAudioOutputSink()) << "The audio output sink did not start and stop its pipeline.";

  EXPECT_EQ(deinitGStreamerCollectingWarnings(), std::vector<std::string>{})
    << "GStreamer objects outlived their pipelines; GST_DEBUG=GST_TRACER:7 lists them above.";
}

}  // namespace
}  // namespace livekit_ros2_bridge
