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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "video_profiling.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kEnvEnabled[] = "LIVEKIT_BRIDGE_VIDEO_PROFILING";
constexpr char kEnvSummaryIntervalMs[] = "LIVEKIT_BRIDGE_VIDEO_PROFILE_SUMMARY_INTERVAL_MS";
constexpr char kEnvTraceFile[] = "LIVEKIT_BRIDGE_VIDEO_TRACE_FILE";
constexpr char kEnvTraceMaxEvents[] = "LIVEKIT_BRIDGE_VIDEO_TRACE_MAX_EVENTS";

class ScopedRclcppInit
{
public:
  ScopedRclcppInit()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~ScopedRclcppInit()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class ScopedEnvVar
{
public:
  ScopedEnvVar(const char * name, std::optional<std::string> value)
  : name_(name)
  , previous_value_(std::getenv(name) == nullptr ? std::nullopt : std::optional<std::string>(std::getenv(name)))
  {
    if (value.has_value()) {
      if (setenv(name, value->c_str(), 1) != 0) {
        throw std::runtime_error("failed to set test environment variable");
      }
    } else {
      if (unsetenv(name) != 0) {
        throw std::runtime_error("failed to unset test environment variable");
      }
    }
  }

  ~ScopedEnvVar()
  {
    if (previous_value_.has_value()) {
      (void)setenv(name_.c_str(), previous_value_->c_str(), 1);
    } else {
      (void)unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> previous_value_;
};

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

std::string readFile(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

}  // namespace

class VideoProfilingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(VideoProfilingTest, LoadConfigDefaultsToDisabledWhenEnvUnset)
{
  const ScopedEnvVar enabled(kEnvEnabled, std::nullopt);
  const ScopedEnvVar interval(kEnvSummaryIntervalMs, std::nullopt);
  const ScopedEnvVar trace_file(kEnvTraceFile, std::nullopt);
  const ScopedEnvVar max_events(kEnvTraceMaxEvents, std::nullopt);

  const VideoProfilingConfig config = loadVideoProfilingConfigFromEnv();

  EXPECT_FALSE(config.requested);
  EXPECT_FALSE(config.enabled);
  EXPECT_EQ(config.summary_interval, kVideoProfilingDefaultSummaryInterval);
  EXPECT_EQ(config.trace_file, kVideoProfilingDefaultTraceFile);
  EXPECT_EQ(config.trace_max_events, kVideoProfilingDefaultTraceMaxEvents);
}

TEST_F(VideoProfilingTest, LoadConfigRejectsInvalidEnabledFlag)
{
  const ScopedEnvVar enabled(kEnvEnabled, std::string("maybe"));

  EXPECT_THROW((void)loadVideoProfilingConfigFromEnv(), std::runtime_error);
}

TEST_F(VideoProfilingTest, LoadConfigReflectsBuildAvailability)
{
  const ScopedEnvVar enabled(kEnvEnabled, std::string("1"));
  const VideoProfilingConfig config = loadVideoProfilingConfigFromEnv();

  EXPECT_TRUE(config.requested);
  EXPECT_EQ(config.build_enabled, kVideoProfilingBuildEnabled);
  EXPECT_EQ(config.enabled, kVideoProfilingBuildEnabled);
}

TEST_F(VideoProfilingTest, RegistryCollectsSummaryAndWritesTrace)
{
  if (!kVideoProfilingBuildEnabled) {
    GTEST_SKIP() << "Video profiling build is disabled.";
  }

  const std::filesystem::path temp_dir =
    std::filesystem::temp_directory_path() / "livekit_ros2_bridge_video_profiling_test";
  const std::filesystem::path trace_path = temp_dir / "trace.json";

  VideoProfilingConfig config;
  config.requested = true;
  config.enabled = true;
  config.trace_file = trace_path.string();
  config.trace_max_events = 32;

  VideoProfilingRegistry registry(rclcpp::get_logger("video_profiling_test"), config);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

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
  EXPECT_EQ(summary.stream_key, "topic:/synthetic/front_camera/image_raw");
  EXPECT_EQ(summary.frames_in, 2U);
  EXPECT_EQ(summary.frames_sampled, 2U);
  EXPECT_EQ(summary.frames_captured, 2U);
  EXPECT_EQ(summary.track_publish_count, 1U);
  EXPECT_TRUE(summary.ingress_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.output_arrival_gap_ms.hasSamples());
  EXPECT_TRUE(summary.source_timestamp_gap_ms.hasSamples());
  EXPECT_TRUE(summary.appsrc_to_sample_ms.hasSamples());
  EXPECT_TRUE(summary.source_to_output_submit_ms.hasSamples());
  EXPECT_TRUE(summary.push_to_appsrc_ms.hasSamples());
  EXPECT_TRUE(summary.sample_callback_ms.hasSamples());

  EXPECT_TRUE(registry.collectAndResetSummaries().empty());

  registry.flushTrace();

  ASSERT_TRUE(std::filesystem::exists(trace_path));
  const std::string trace_contents = readFile(trace_path);
  EXPECT_NE(trace_contents.find("\"traceEvents\""), std::string::npos);
  EXPECT_NE(trace_contents.find("stream.registered"), std::string::npos);
  EXPECT_NE(trace_contents.find("source.queue_to_gst_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("source_to_sample_ready_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("source_to_livekit_submit_ms"), std::string::npos);
  EXPECT_NE(trace_contents.find("livekit.frame_submitted"), std::string::npos);
}

TEST_F(VideoProfilingTest, RegistryTracksDroppedTraceEvents)
{
  if (!kVideoProfilingBuildEnabled) {
    GTEST_SKIP() << "Video profiling build is disabled.";
  }

  VideoProfilingConfig config;
  config.requested = true;
  config.enabled = true;
  config.trace_max_events = 2;

  VideoProfilingRegistry registry(rclcpp::get_logger("video_profiling_drop_test"), config);
  const auto profiler = registry.getOrCreateStreamProfiler(makeSpec());
  ASSERT_NE(profiler, nullptr);

  profiler->notePipelineStart();
  profiler->notePipelineFailure("failure_one");
  profiler->notePipelineFailure("failure_two");
  profiler->notePipelineFailure("failure_three");

  EXPECT_GT(registry.traceDroppedEventCount(), 0U);
}

}  // namespace livekit_ros2_bridge
