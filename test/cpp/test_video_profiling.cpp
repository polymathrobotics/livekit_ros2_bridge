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

#include <array>
#include <filesystem>
#include <fstream>
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

VideoProfilingRegistry makeRegistry(const std::filesystem::path & trace_path, std::size_t trace_max_events = 32U)
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

constexpr std::array<VideoProfileStage, 7> kStages{{
  VideoProfileStage::kPushToAppSrc,
  VideoProfileStage::kSampleCallback,
  VideoProfileStage::kSampleUnpack,
  VideoProfileStage::kFrameSink,
  VideoProfileStage::kPublisherHandleFrame,
  VideoProfileStage::kEnsureTrack,
  VideoProfileStage::kCaptureFrame,
}};

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
  EXPECT_EQ(registry.getOrCreateProfiler(makeSpec()), nullptr);
  registry.flushTrace();

  EXPECT_FALSE(std::filesystem::exists(trace_path));
}

TEST(VideoProfilingTest, RegistryCollectsOnlyActiveStreamSummariesAndResetsThem)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_summary_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto active_spec = makeSpec();
  auto idle_spec = makeSpec();
  idle_spec.stream_key = "topic:/synthetic/rear_camera/image_raw";
  idle_spec.track_name = "ros.video.synthetic.rear_camera";
  idle_spec.ros_topic = "/synthetic/rear_camera/image_raw";

  const auto profiler = registry.getOrCreateProfiler(active_spec);
  const auto idle_profiler = registry.getOrCreateProfiler(idle_spec);
  ASSERT_NE(profiler, nullptr);
  ASSERT_NE(idle_profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngress(base, 1000);
  profiler->noteIngress(base + std::chrono::milliseconds(33), 34000);
  profiler->noteSample(1000);
  profiler->noteCapture(1000);
  profiler->recordStage(
    VideoProfileStage::kPushToAppSrc, std::chrono::microseconds(1500), 34000, base + std::chrono::milliseconds(33));
  profiler->noteSample(34000);
  profiler->recordStage(
    VideoProfileStage::kSampleCallback, std::chrono::microseconds(2400), 34000, base + std::chrono::milliseconds(33));
  profiler->noteTrackPublished(640, 360, false);
  profiler->noteCapture(34000);

  const auto summaries = registry.takeSummaries();
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
  EXPECT_TRUE(registry.takeSummaries().empty());
}

TEST(VideoProfilingTest, RegistryReusesExistingProfilerForStreamKeyDespiteDifferentRequestedIdentity)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_reused_stream_key_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto active_spec = makeSpec();
  auto reused_spec = makeSpec();
  reused_spec.track_name = "ros.video.other.front_camera";
  reused_spec.input_kind = VideoInputKind::OtherVideoSource;
  reused_spec.ingest_mode = kOtherVideoIngestMode;
  reused_spec.other_video_source_name = "front_camera";

  const auto profiler = registry.getOrCreateProfiler(active_spec);
  const auto reused_profiler = registry.getOrCreateProfiler(reused_spec);
  ASSERT_NE(profiler, nullptr);
  ASSERT_EQ(reused_profiler, profiler);

  reused_profiler->notePipelineStart();

  const auto summaries = registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.stream_key, active_spec.stream_key);
  EXPECT_EQ(summary.track_name, active_spec.track_name);
  EXPECT_EQ(summary.input_kind, videoInputKindToString(active_spec.input_kind));
  EXPECT_EQ(summary.ingest_mode, active_spec.ingest_mode);
  EXPECT_EQ(summary.pipeline_start_count, 1U);
}

TEST(VideoProfilingTest, SummaryTracksTimestampRejectionsWithoutRecordingLatencyMetrics)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_timestamp_rejection_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngress(base, 1000);
  profiler->noteIngress(base + std::chrono::milliseconds(10), 2000);
  profiler->noteIngress(base + std::chrono::milliseconds(20), 1500);
  profiler->noteSample(1000);
  profiler->noteCapture(1000);
  profiler->noteIngress(base + std::chrono::milliseconds(30), -1);
  profiler->noteSample(-1);
  profiler->noteCapture(-1);

  const auto summaries = registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.source_timestamp_regression_count, 2U);
  EXPECT_TRUE(summary.ingress_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.source_timestamp_gap_ms.hasSamples());
  EXPECT_FALSE(summary.appsrc_to_sample_ms.hasSamples());
  EXPECT_FALSE(summary.source_to_output_submit_ms.hasSamples());
}

TEST(VideoProfilingTest, SampleKeepsIngressMatchAvailableUntilCaptureConsumesIt)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_match_lifecycle_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngress(base, 1000);
  profiler->noteSample(1000);
  profiler->noteCapture(1000);
  profiler->noteCapture(1000);

  const auto summaries = registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
  EXPECT_EQ(summary.appsrc_to_sample_ms.sample_count, 1U);
  EXPECT_EQ(summary.source_to_output_submit_ms.sample_count, 1U);
}

TEST(VideoProfilingTest, SummaryBoundaryPreservesPendingIngressCorrelationWithoutExtraSummary)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_boundary_correlation_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto base = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngress(base, 1000);

  const auto ingress_only_summaries = registry.takeSummaries();
  ASSERT_EQ(ingress_only_summaries.size(), 1U);
  EXPECT_EQ(ingress_only_summaries.front().frames_in, 1U);
  EXPECT_TRUE(registry.takeSummaries().empty());

  profiler->noteSample(1000);
  profiler->noteCapture(1000);

  const auto matched_summaries = registry.takeSummaries();
  ASSERT_EQ(matched_summaries.size(), 1U);
  const auto & summary = matched_summaries.front();
  EXPECT_EQ(summary.frames_in, 0U);
  EXPECT_EQ(summary.frames_sampled, 1U);
  EXPECT_EQ(summary.frames_captured, 1U);
  EXPECT_TRUE(summary.appsrc_to_sample_ms.hasSamples());
  EXPECT_TRUE(summary.source_to_output_submit_ms.hasSamples());
}

TEST(VideoProfilingTest, SummaryCapturesFailureLifecycleAndStageMetrics)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_failure_lifecycle_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  profiler->notePipelineFailure("pipeline_failure");
  profiler->noteRestartFailed("restart_failed");
  profiler->notePushFailed("push_failed");
  profiler->noteSampleUnpackFailed("sample_unpack_failed");
  profiler->noteCaptureFailed("capture_failed");
  profiler->noteTrackPublished(1280, 720, false);
  profiler->noteTrackPublished(1280, 720, true);
  profiler->noteTrackUnpublish();
  profiler->recordStage(VideoProfileStage::kSampleUnpack, std::chrono::microseconds(500));
  profiler->recordStage(VideoProfileStage::kFrameSink, std::chrono::microseconds(600));
  profiler->recordStage(VideoProfileStage::kPublisherHandleFrame, std::chrono::microseconds(700));
  profiler->recordStage(VideoProfileStage::kEnsureTrack, std::chrono::microseconds(800));
  profiler->recordStage(VideoProfileStage::kCaptureFrame, std::chrono::microseconds(900));

  const auto summaries = registry.takeSummaries();
  ASSERT_EQ(summaries.size(), 1U);
  const auto & summary = summaries.front();
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
  VideoProfilingRegistry registry = makeRegistry(trace_path);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  const auto ingress_time = VideoStreamProfiler::SteadyClock::now();
  profiler->noteIngress(ingress_time, 1000);
  profiler->noteSample(1000);
  profiler->noteCapture(1000);
  std::int64_t stage_duration_us = 1500;
  for (const auto stage : kStages) {
    profiler->recordStage(stage, std::chrono::microseconds(stage_duration_us), 1000, ingress_time);
    stage_duration_us += 100;
  }
  profiler->noteTrackPublished(640, 360, false);

  registry.flushTrace();

  std::ifstream input(trace_path);
  ASSERT_TRUE(input.is_open());
  std::stringstream buffer;
  buffer << input.rdbuf();
  const std::string trace_contents = buffer.str();
  EXPECT_NE(trace_contents.find("\"traceEvents\""), std::string::npos);
  EXPECT_NE(trace_contents.find("stream.registered"), std::string::npos);
  for (const auto stage : kStages) {
    EXPECT_NE(trace_contents.find(videoProfileStageToString(stage)), std::string::npos);
  }
  EXPECT_NE(trace_contents.find("source_to_sample_ready_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("source_to_livekit_submit_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("livekit.frame_submitted"), std::string::npos);
}

TEST(VideoProfilingTest, RegistryTracksDroppedTraceEvents)
{
  test_support::ScopedRclcppInit init;
  const auto trace_path = makeTracePath("livekit_ros2_bridge_video_profiling_drop_trace.json");
  VideoProfilingRegistry registry = makeRegistry(trace_path, 2U);
  const auto profiler = registry.getOrCreateProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  profiler->notePipelineFailure("failure_one");
  profiler->notePipelineFailure("failure_two");
  profiler->notePipelineFailure("failure_three");

  EXPECT_GT(registry.traceDroppedEventCount(), 0U);
}

}  // namespace livekit_ros2_bridge
