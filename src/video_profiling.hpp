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

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

inline constexpr char kVideoProfilingDefaultTraceFile[] = "/workspace/log/video-profile.trace.json";
inline constexpr std::size_t kVideoProfilingDefaultTraceMaxEvents = 250000U;
inline constexpr auto kVideoProfilingDefaultSummaryInterval = std::chrono::milliseconds(5000);

struct VideoProfilingConfig
{
  bool enabled = false;
  std::chrono::milliseconds summary_interval = kVideoProfilingDefaultSummaryInterval;
  std::string trace_file = kVideoProfilingDefaultTraceFile;
  std::size_t trace_max_events = kVideoProfilingDefaultTraceMaxEvents;
};

enum class VideoProfileStage
{
  kPushToAppSrc,
  kSampleCallback,
  kSampleUnpack,
  kFrameSink,
  kPublisherHandleFrame,
  kEnsureTrack,
  kCaptureFrame,
};

const char * videoProfileStageToString(VideoProfileStage stage);

struct VideoProfileDurationSummary
{
  std::size_t sample_count = 0;
  double avg_ms = 0.0;
  double p95_ms = 0.0;
  double max_ms = 0.0;

  bool hasSamples() const
  {
    return sample_count > 0;
  }
};

struct VideoStreamProfileSummary
{
  std::string stream_key;
  std::string track_name;
  std::string input_kind;
  std::string ingest_mode;

  std::size_t frames_in = 0;
  std::size_t frames_sampled = 0;
  std::size_t frames_captured = 0;

  std::size_t pipeline_start_count = 0;
  std::size_t pipeline_failure_count = 0;
  std::size_t restart_failed_count = 0;
  std::size_t push_failed_count = 0;
  std::size_t sample_unpack_failed_count = 0;
  std::size_t capture_failed_count = 0;
  std::size_t track_publish_count = 0;
  std::size_t track_republish_count = 0;
  std::size_t track_unpublish_count = 0;
  std::size_t source_timestamp_regression_count = 0;

  VideoProfileDurationSummary ingress_arrival_gap_ms;
  VideoProfileDurationSummary output_arrival_gap_ms;
  VideoProfileDurationSummary source_timestamp_gap_ms;
  VideoProfileDurationSummary appsrc_to_sample_ms;
  VideoProfileDurationSummary source_to_output_submit_ms;
  VideoProfileDurationSummary push_to_appsrc_ms;
  VideoProfileDurationSummary sample_callback_ms;
  VideoProfileDurationSummary sample_unpack_ms;
  VideoProfileDurationSummary frame_sink_ms;
  VideoProfileDurationSummary publisher_handle_frame_ms;
  VideoProfileDurationSummary ensure_track_ms;
  VideoProfileDurationSummary capture_frame_ms;

  bool hasActivity() const;
};

class VideoStreamProfiler final
{
public:
  using SteadyClock = std::chrono::steady_clock;

  class ScopedStageTimer final
  {
  public:
    ScopedStageTimer() = default;
    ScopedStageTimer(
      VideoStreamProfiler * profiler,
      VideoProfileStage stage,
      std::optional<std::int64_t> frame_timestamp_us = std::nullopt);
    ~ScopedStageTimer();

    ScopedStageTimer(const ScopedStageTimer &) = delete;
    ScopedStageTimer & operator=(const ScopedStageTimer &) = delete;
    ScopedStageTimer(ScopedStageTimer && other) noexcept;
    ScopedStageTimer & operator=(ScopedStageTimer && other) noexcept;

  private:
    VideoStreamProfiler * profiler_ = nullptr;
    VideoProfileStage stage_ = VideoProfileStage::kPushToAppSrc;
    std::optional<std::int64_t> frame_timestamp_us_;
    SteadyClock::time_point start_time_{};
  };

  explicit VideoStreamProfiler(VideoStreamSpec spec);
  ~VideoStreamProfiler();

  VideoStreamProfiler(const VideoStreamProfiler &) = delete;
  VideoStreamProfiler & operator=(const VideoStreamProfiler &) = delete;
  VideoStreamProfiler(VideoStreamProfiler &&) = delete;
  VideoStreamProfiler & operator=(VideoStreamProfiler &&) = delete;

  void noteIngressFrame(SteadyClock::time_point arrival_time, std::optional<std::int64_t> source_timestamp_us);
  void noteSampledFrame(std::optional<std::int64_t> frame_timestamp_us = std::nullopt);
  void noteFrameCaptured(std::optional<std::int64_t> frame_timestamp_us = std::nullopt);

  void notePipelineStart();
  void notePipelineFailure(const std::string & reason);
  void noteRestartFailed(const std::string & error);
  void notePushFailed(const std::string & error);
  void noteSampleUnpackFailed(const std::string & error);
  void noteCaptureFailed(const std::string & error);
  void noteTrackPublished(int width, int height, bool republished);
  void noteTrackUnpublishing();

  void recordStageDuration(
    VideoProfileStage stage,
    std::chrono::microseconds duration,
    std::optional<std::int64_t> frame_timestamp_us = std::nullopt,
    std::optional<SteadyClock::time_point> start_time = std::nullopt);

  std::optional<VideoStreamProfileSummary> collectAndResetSummary();

private:
  friend class VideoProfilingRegistry;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class VideoProfilingRegistry final
{
public:
  explicit VideoProfilingRegistry(rclcpp::Logger logger, VideoProfilingConfig config);
  ~VideoProfilingRegistry();

  VideoProfilingRegistry(const VideoProfilingRegistry &) = delete;
  VideoProfilingRegistry & operator=(const VideoProfilingRegistry &) = delete;
  VideoProfilingRegistry(VideoProfilingRegistry &&) = delete;
  VideoProfilingRegistry & operator=(VideoProfilingRegistry &&) = delete;

  bool enabled() const;
  const VideoProfilingConfig & config() const;

  std::shared_ptr<VideoStreamProfiler> getOrCreateStreamProfiler(const VideoStreamSpec & spec);
  std::vector<VideoStreamProfileSummary> collectAndResetSummaries();
  std::size_t traceDroppedEventCount() const;
  void emitEnabledLog() const;
  void emitSummaryLogs();
  void flushTrace();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livekit_ros2_bridge
