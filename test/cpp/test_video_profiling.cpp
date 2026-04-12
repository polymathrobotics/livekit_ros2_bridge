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

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "video_profiling.hpp"

namespace livekit_ros2_bridge
{

namespace
{

VideoStreamSpec makeSpec()
{
  VideoStreamSpec spec;
  spec.stream_key = "topic:/synthetic/front_camera/image_raw";
  spec.track_name = "ros.video.synthetic.front_camera";
  spec.ros_topic = "/synthetic/front_camera/image_raw";
  spec.input_kind = VideoInputKind::RosTopic;
  spec.ingest_mode = kRawImageIngestMode;
  return spec;
}

VideoProfilingRegistry makeEnabledRegistry(const std::filesystem::path & trace_path, std::size_t trace_max_events = 32U)
{
  VideoProfilingConfig config;
  config.enabled = true;
  config.trace_file = trace_path.string();
  config.trace_max_events = trace_max_events;
  return VideoProfilingRegistry(rclcpp::get_logger("video_profiling_test"), config);
}

std::filesystem::path makeTracePath(const char * filename)
{
  return std::filesystem::temp_directory_path() / filename;
}

}  // namespace

TEST(VideoProfilingTest, DefaultConfigStartsDisabled)
{
  const VideoProfilingConfig config;

  EXPECT_FALSE(config.enabled);
  EXPECT_EQ(config.summary_interval, kVideoProfilingDefaultSummaryInterval);
  EXPECT_EQ(config.trace_file, kVideoProfilingDefaultTraceFile);
  EXPECT_EQ(config.trace_max_events, kVideoProfilingDefaultTraceMaxEvents);
}

TEST(VideoProfilingTest, DisabledRegistryRejectsProfilerCreationAndDoesNotWriteTrace)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_disabled_trace.json");
  std::filesystem::remove(trace_path);

  VideoProfilingConfig config;
  config.trace_file = trace_path.string();
  VideoProfilingRegistry registry(rclcpp::get_logger("video_profiling_test"), config);

  EXPECT_FALSE(registry.enabled());
  EXPECT_EQ(registry.getOrCreateStreamProfiler(makeSpec()), nullptr);
  registry.flushTrace();

  EXPECT_FALSE(std::filesystem::exists(trace_path));
}

TEST(VideoProfilingTest, RegistryCollectsOnlyActiveStreamSummariesAndResetsThem)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_summary_trace.json");
  VideoProfilingRegistry registry = makeEnabledRegistry(trace_path);
  const auto active_spec = makeSpec();
  auto idle_spec = makeSpec();
  idle_spec.stream_key = "topic:/synthetic/rear_camera/image_raw";
  idle_spec.track_name = "ros.video.synthetic.rear_camera";
  idle_spec.ros_topic = "/synthetic/rear_camera/image_raw";

  const auto profiler = registry.getOrCreateStreamProfiler(active_spec);
  const auto idle_profiler = registry.getOrCreateStreamProfiler(idle_spec);
  ASSERT_NE(profiler, nullptr);
  ASSERT_NE(idle_profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngressFrame(base, 1000);
  profiler->noteIngressFrame(base + std::chrono::milliseconds(33), 34000);
  profiler->noteSampledFrame(1000);
  profiler->noteFrameCaptured(1000);
  profiler->recordStageDuration(
    VideoProfileStage::kPushToAppSrc, std::chrono::microseconds(1500), 34000, base + std::chrono::milliseconds(33));
  profiler->noteSampledFrame(34000);
  profiler->recordStageDuration(
    VideoProfileStage::kSampleCallback, std::chrono::microseconds(2400), 34000, base + std::chrono::milliseconds(33));
  profiler->noteTrackPublished(640, 360, false);
  profiler->noteFrameCaptured(34000);

  const auto summaries = registry.collectAndResetSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.stream_key, active_spec.stream_key);
  EXPECT_EQ(summary.track_name, active_spec.track_name);
  EXPECT_EQ(summary.frames_in, 2U);
  EXPECT_EQ(summary.frames_sampled, 2U);
  EXPECT_EQ(summary.frames_captured, 2U);
  EXPECT_TRUE(summary.ingress_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.output_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.source_timestamp_gap_ms.hasSamples());
  EXPECT_TRUE(summary.appsrc_to_sample_ms.hasSamples());
  EXPECT_TRUE(summary.source_to_output_submit_ms.hasSamples());
  EXPECT_TRUE(summary.push_to_appsrc_ms.hasSamples());
  EXPECT_TRUE(summary.sample_callback_ms.hasSamples());
  EXPECT_TRUE(registry.collectAndResetSummaries().empty());
}

TEST(VideoProfilingTest, SummaryTracksTimestampRejectionsWithoutRecordingLatencyMetrics)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_timestamp_rejection_trace.json");
  VideoProfilingRegistry registry = makeEnabledRegistry(trace_path);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngressFrame(base, 1000);
  profiler->noteIngressFrame(base + std::chrono::milliseconds(10), 2000);
  profiler->noteIngressFrame(base + std::chrono::milliseconds(20), 1500);
  profiler->noteSampledFrame(1000);
  profiler->noteFrameCaptured(1000);
  profiler->noteIngressFrame(base + std::chrono::milliseconds(30), -1);
  profiler->noteSampledFrame(-1);
  profiler->noteFrameCaptured(-1);

  const auto summaries = registry.collectAndResetSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.frames_in, 4U);
  EXPECT_EQ(summary.frames_sampled, 2U);
  EXPECT_EQ(summary.frames_captured, 2U);
  EXPECT_EQ(summary.source_timestamp_regression_count, 2U);
  EXPECT_TRUE(summary.ingress_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.source_timestamp_gap_ms.hasSamples());
  EXPECT_FALSE(summary.appsrc_to_sample_ms.hasSamples());
  EXPECT_FALSE(summary.source_to_output_submit_ms.hasSamples());
}

TEST(VideoProfilingTest, SummaryCapturesFailureLifecycleAndStageMetrics)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_failure_lifecycle_trace.json");
  VideoProfilingRegistry registry = makeEnabledRegistry(trace_path);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  profiler->notePipelineStart();
  profiler->notePipelineFailure("pipeline_failure");
  profiler->noteRestartFailed("restart_failed");
  profiler->notePushFailed("push_failed");
  profiler->noteSampleUnpackFailed("sample_unpack_failed");
  profiler->noteCaptureFailed("capture_failed");
  profiler->noteTrackPublished(1280, 720, false);
  profiler->noteTrackPublished(1280, 720, true);
  profiler->noteTrackUnpublishing();
  profiler->recordStageDuration(VideoProfileStage::kSampleUnpack, std::chrono::microseconds(500));
  profiler->recordStageDuration(VideoProfileStage::kFrameSink, std::chrono::microseconds(600));
  profiler->recordStageDuration(VideoProfileStage::kPublisherHandleFrame, std::chrono::microseconds(700));
  profiler->recordStageDuration(VideoProfileStage::kEnsureTrack, std::chrono::microseconds(800));
  profiler->recordStageDuration(VideoProfileStage::kCaptureFrame, std::chrono::microseconds(900));

  const auto summaries = registry.collectAndResetSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.pipeline_start_count, 1U);
  EXPECT_EQ(summary.pipeline_failure_count, 1U);
  EXPECT_EQ(summary.restart_failed_count, 1U);
  EXPECT_EQ(summary.push_failed_count, 1U);
  EXPECT_EQ(summary.sample_unpack_failed_count, 1U);
  EXPECT_EQ(summary.capture_failed_count, 1U);
  EXPECT_EQ(summary.track_publish_count, 1U);
  EXPECT_EQ(summary.track_republish_count, 1U);
  EXPECT_EQ(summary.track_unpublish_count, 1U);
  EXPECT_TRUE(summary.sample_unpack_ms.hasSamples());
  EXPECT_TRUE(summary.frame_sink_ms.hasSamples());
  EXPECT_TRUE(summary.publisher_handle_frame_ms.hasSamples());
  EXPECT_TRUE(summary.ensure_track_ms.hasSamples());
  EXPECT_TRUE(summary.capture_frame_ms.hasSamples());
}

TEST(VideoProfilingTest, RegistryFlushTraceWritesExpectedEvents)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_trace.json");
  VideoProfilingRegistry registry = makeEnabledRegistry(trace_path);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto ingress_time = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngressFrame(ingress_time, 1000);
  profiler->noteSampledFrame(1000);
  profiler->noteFrameCaptured(1000);
  profiler->recordStageDuration(VideoProfileStage::kPushToAppSrc, std::chrono::microseconds(1500), 1000, ingress_time);
  profiler->recordStageDuration(
    VideoProfileStage::kSampleCallback, std::chrono::microseconds(2400), 1000, ingress_time);
  profiler->noteTrackPublished(640, 360, false);

  registry.flushTrace();

  std::ifstream input(trace_path);
  ASSERT_TRUE(input.is_open());
  std::stringstream buffer;
  buffer << input.rdbuf();
  const std::string trace_contents = buffer.str();
  EXPECT_NE(trace_contents.find("\"traceEvents\""), std::string::npos);
  EXPECT_NE(trace_contents.find("stream.registered"), std::string::npos);
  EXPECT_NE(trace_contents.find("source.queue_to_gst_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("source_to_sample_ready_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("source_to_livekit_submit_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("livekit.frame_submitted"), std::string::npos);
}

TEST(VideoProfilingTest, RegistryTracksDroppedTraceEvents)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_drop_trace.json");
  VideoProfilingRegistry registry = makeEnabledRegistry(trace_path, 2U);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  profiler->notePipelineFailure("failure_one");
  profiler->notePipelineFailure("failure_two");
  profiler->notePipelineFailure("failure_three");

  EXPECT_GT(registry.traceDroppedEventCount(), 0U);
}

}  // namespace livekit_ros2_bridge
