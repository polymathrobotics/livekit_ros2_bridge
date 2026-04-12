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

#include "video_profiling.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using SteadyClock = VideoStreamProfiler::SteadyClock;
using TraceArgValue = std::variant<std::int64_t, double, bool, std::string>;

enum class TraceEventKind
{
  kComplete,
  kInstant,
};

struct TraceArg
{
  std::string key;
  TraceArgValue value;
};

struct TraceEvent
{
  TraceEventKind kind = TraceEventKind::kInstant;
  std::string category;
  std::string name;
  std::int64_t timestamp_us = 0;
  std::int64_t duration_us = 0;
  std::uint64_t thread_id = 0;
  std::vector<TraceArg> args;
};

struct MetricSamples
{
  std::vector<double> values_ms;

  void addMilliseconds(double value_ms)
  {
    values_ms.push_back(value_ms);
  }

  VideoProfileDurationSummary summarizeAndReset()
  {
    VideoProfileDurationSummary summary;
    summary.sample_count = values_ms.size();
    if (values_ms.empty()) {
      return summary;
    }

    const double sum_ms = std::accumulate(values_ms.begin(), values_ms.end(), 0.0);
    summary.avg_ms = sum_ms / static_cast<double>(values_ms.size());

    std::sort(values_ms.begin(), values_ms.end());
    const std::size_t p95_index =
      static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values_ms.size()))) - 1U;
    summary.p95_ms = values_ms[std::min(p95_index, values_ms.size() - 1U)];
    summary.max_ms = values_ms.back();
    values_ms.clear();
    return summary;
  }
};

std::uint64_t currentThreadId()
{
  return static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

std::string toUtcIso8601(std::chrono::system_clock::time_point time_point)
{
  const std::time_t time_value = std::chrono::system_clock::to_time_t(time_point);
  std::tm utc_time{};
#if defined(_WIN32)
  gmtime_s(&utc_time, &time_value);
#else
  gmtime_r(&time_value, &utc_time);
#endif
  std::ostringstream output;
  output << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

nlohmann::json traceArgToJson(const TraceArgValue & value)
{
  return std::visit([](const auto & typed_value) -> nlohmann::json { return typed_value; }, value);
}

void addTraceArg(std::vector<TraceArg> & args, const char * key, std::int64_t value)
{
  args.push_back({key, value});
}

void addTraceArg(std::vector<TraceArg> & args, const char * key, std::string value)
{
  args.push_back({key, std::move(value)});
}

void addTraceArg(std::vector<TraceArg> & args, const char * key, bool value)
{
  args.push_back({key, value});
}

void addMetricFields(LogEvent & event, const char * field_prefix, const VideoProfileDurationSummary & metric)
{
  if (!metric.hasSamples()) {
    return;
  }

  const std::string base(field_prefix);
  event.field(base + "_samples", metric.sample_count)
    .field(base + "_avg", metric.avg_ms)
    .field(base + "_p95", metric.p95_ms)
    .field(base + "_max", metric.max_ms);
}

const char * ingressTraceEventName()
{
  return "source.received";
}

const char * sampledTraceEventName()
{
  return "gst.sample_ready";
}

const char * capturedTraceEventName()
{
  return "livekit.frame_submitted";
}

const char * pipelineStartTraceEventName()
{
  return "gst.pipeline_started";
}

const char * pipelineFailureTraceEventName()
{
  return "gst.pipeline_failed";
}

const char * restartFailureTraceEventName()
{
  return "gst.restart_failed";
}

const char * pushFailureTraceEventName()
{
  return "source.queue_to_gst_failed";
}

const char * sampleUnpackFailureTraceEventName()
{
  return "gst.repack_failed";
}

const char * captureFailureTraceEventName()
{
  return "livekit.submit_failed";
}

const char * sourceToSampleReadyTraceEventName()
{
  return "source_to_sample_ready_ms";
}

const char * sourceToLiveKitSubmitTraceEventName()
{
  return "source_to_livekit_submit_ms";
}

const char * trackPublishedTraceEventName(bool republished)
{
  return republished ? "publish.track_republished" : "publish.track_published";
}

const char * trackUnpublishingTraceEventName()
{
  return "publish.track_unpublishing";
}

const char * streamRegisteredTraceEventName()
{
  return "stream.registered";
}

const char * summaryFramesInFieldName()
{
  return "source.received_count";
}

const char * summaryFramesSampledFieldName()
{
  return "gst.sample_count";
}

const char * summaryFramesCapturedFieldName()
{
  return "livekit.frame_submitted_count";
}

const char * summaryPipelineStartCountFieldName()
{
  return "gst.pipeline_started_count";
}

const char * summaryPipelineFailureCountFieldName()
{
  return "gst.pipeline_failed_count";
}

const char * summaryRestartFailedCountFieldName()
{
  return "gst.restart_failed_count";
}

const char * summaryPushFailedCountFieldName()
{
  return "source.queue_to_gst_failed_count";
}

const char * summarySampleUnpackFailedCountFieldName()
{
  return "gst.repack_failed_count";
}

const char * summaryCaptureFailedCountFieldName()
{
  return "livekit.submit_failed_count";
}

const char * summaryTrackPublishCountFieldName()
{
  return "publish.track_published_count";
}

const char * summaryTrackRepublishCountFieldName()
{
  return "publish.track_republished_count";
}

const char * summaryTrackUnpublishCountFieldName()
{
  return "publish.track_unpublishing_count";
}

const char * summarySourceTimestampRegressionCountFieldName()
{
  return "source.timestamp_regression_count";
}

const char * summaryIngressArrivalGapFieldName()
{
  return "source.interarrival_gap_ms";
}

const char * summarySourceTimestampGapFieldName()
{
  return "source.timestamp_gap_ms";
}

const char * summaryOutputArrivalGapFieldName()
{
  return "livekit.interarrival_gap_ms";
}

const char * summarySourceToOutputSubmitFieldName()
{
  return "source_to_livekit_submit_ms";
}

const char * summarySourceToSampleReadyFieldName()
{
  return "source_to_sample_ready_ms";
}

const char * summaryPushToAppSrcFieldName()
{
  return "source.queue_to_gst_ms";
}

const char * summarySampleCallbackFieldName()
{
  return "gst.sample_total_ms";
}

const char * summarySampleUnpackFieldName()
{
  return "gst.repack_i420_ms";
}

const char * summaryFrameSinkFieldName()
{
  return "bridge.to_publisher_ms";
}

const char * summaryPublisherHandleFieldName()
{
  return "publish.total_ms";
}

const char * summaryEnsureTrackFieldName()
{
  return "publish.ensure_track_ms";
}

const char * summaryCaptureFrameFieldName()
{
  return "livekit.submit_ms";
}

class VideoTraceRecorder final
{
public:
  explicit VideoTraceRecorder(const VideoProfilingConfig & config)
  : trace_file_(config.trace_file)
  , max_events_(config.trace_max_events)
  , steady_origin_(SteadyClock::now())
  , system_origin_(std::chrono::system_clock::now())
  , system_origin_unix_ms_(
      std::chrono::duration_cast<std::chrono::milliseconds>(system_origin_.time_since_epoch()).count())
  , system_origin_utc_(toUtcIso8601(system_origin_))
  , process_id_(::getpid())
  {
    std::vector<TraceArg> args;
    addTraceArg(args, "startup_unix_time_ms", system_origin_unix_ms_);
    addTraceArg(args, "startup_utc", system_origin_utc_);
    addTraceArg(args, "trace_file", trace_file_);
    addTraceArg(args, "trace_max_events", static_cast<std::int64_t>(max_events_));
    recordInstant("video_profile", "video_profiling_startup", steady_origin_, std::move(args));
  }

  void recordComplete(
    std::string category,
    std::string name,
    SteadyClock::time_point start_time,
    std::chrono::microseconds duration,
    std::vector<TraceArg> args = {})
  {
    TraceEvent event;
    event.kind = TraceEventKind::kComplete;
    event.category = std::move(category);
    event.name = std::move(name);
    event.timestamp_us = toTraceTimestampUs(start_time);
    event.duration_us = duration.count();
    event.thread_id = currentThreadId();
    event.args = std::move(args);
    appendEvent(std::move(event));
  }

  void recordInstant(
    std::string category, std::string name, SteadyClock::time_point event_time, std::vector<TraceArg> args = {})
  {
    TraceEvent event;
    event.kind = TraceEventKind::kInstant;
    event.category = std::move(category);
    event.name = std::move(name);
    event.timestamp_us = toTraceTimestampUs(event_time);
    event.thread_id = currentThreadId();
    event.args = std::move(args);
    appendEvent(std::move(event));
  }

  std::size_t droppedEventCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_event_count_;
  }

  std::size_t writeTraceFile() const
  {
    std::deque<TraceEvent> events;
    std::vector<std::uint64_t> thread_ids;
    std::size_t dropped_event_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events = events_;
      thread_ids.assign(thread_ids_.begin(), thread_ids_.end());
      dropped_event_count = dropped_event_count_;
    }

    nlohmann::json trace_events = nlohmann::json::array();
    trace_events.push_back(
      {{"name", "process_name"},
       {"ph", "M"},
       {"pid", process_id_},
       {"tid", 0},
       {"args", {{"name", "livekit_ros2_bridge"}}}});

    for (const std::uint64_t thread_id : thread_ids) {
      trace_events.push_back(
        {{"name", "thread_name"},
         {"ph", "M"},
         {"pid", process_id_},
         {"tid", thread_id},
         {"args", {{"name", "thread_" + std::to_string(thread_id)}}}});
    }

    for (const auto & event : events) {
      nlohmann::json args = nlohmann::json::object();
      for (const auto & arg : event.args) {
        args[arg.key] = traceArgToJson(arg.value);
      }

      nlohmann::json json_event{
        {"name", event.name},
        {"cat", event.category},
        {"pid", process_id_},
        {"tid", event.thread_id},
        {"ts", event.timestamp_us},
        {"args", std::move(args)},
      };
      if (event.kind == TraceEventKind::kComplete) {
        json_event["ph"] = "X";
        json_event["dur"] = event.duration_us;
      } else {
        json_event["ph"] = "i";
        json_event["s"] = "t";
      }
      trace_events.push_back(std::move(json_event));
    }

    const nlohmann::json root{
      {"displayTimeUnit", "ms"},
      {"otherData",
       {
         {"startup_unix_time_ms", system_origin_unix_ms_},
         {"startup_utc", system_origin_utc_},
         {"dropped_event_count", dropped_event_count},
       }},
      {"traceEvents", std::move(trace_events)},
    };

    const std::filesystem::path trace_path(trace_file_);
    if (trace_path.has_parent_path()) {
      std::filesystem::create_directories(trace_path.parent_path());
    }
    std::ofstream output(trace_path);
    if (!output.is_open()) {
      throw std::runtime_error("failed to open trace file '" + trace_file_ + "' for writing");
    }
    output << root.dump();
    output << std::endl;
    return events.size();
  }

private:
  std::int64_t toTraceTimestampUs(SteadyClock::time_point time_point) const
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(time_point - steady_origin_).count();
  }

  void appendEvent(TraceEvent event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (max_events_ == 0U) {
      dropped_event_count_++;
      return;
    }

    thread_ids_.push_back(event.thread_id);
    if (events_.size() >= max_events_) {
      events_.pop_front();
      dropped_event_count_++;
    }
    events_.push_back(std::move(event));
  }

  std::string trace_file_;
  std::size_t max_events_ = 0;
  SteadyClock::time_point steady_origin_;
  std::chrono::system_clock::time_point system_origin_;
  std::int64_t system_origin_unix_ms_ = 0;
  std::string system_origin_utc_;
  int process_id_ = 0;

  mutable std::mutex mutex_;
  mutable std::deque<TraceEvent> events_;
  mutable std::vector<std::uint64_t> thread_ids_;
  mutable std::size_t dropped_event_count_ = 0;
};

VideoProfileDurationSummary summarizeAndResetMetric(MetricSamples & metric)
{
  return metric.summarizeAndReset();
}

std::optional<std::int64_t> validateSourceTimestampUs(std::optional<std::int64_t> source_timestamp_us)
{
  if (!source_timestamp_us.has_value()) {
    return std::nullopt;
  }
  if (*source_timestamp_us < 0) {
    return std::nullopt;
  }
  return source_timestamp_us;
}

}  // namespace

struct VideoStreamProfiler::Impl
{
  explicit Impl(VideoStreamSpec stream_spec)
  : spec(std::move(stream_spec))
  {}

  void markActivityLocked()
  {
    activity_since_last_summary = true;
  }

  void recordTraceInstant(const char * name, std::vector<TraceArg> args = {})
  {
    if (trace_recorder == nullptr) {
      return;
    }
    trace_recorder->recordInstant(spec.stream_key, name, SteadyClock::now(), std::move(args));
  }

  void recordTraceComplete(
    const char * name,
    SteadyClock::time_point start_time,
    std::chrono::microseconds duration,
    std::vector<TraceArg> args = {})
  {
    if (trace_recorder == nullptr) {
      return;
    }
    trace_recorder->recordComplete(spec.stream_key, name, start_time, duration, std::move(args));
  }

  void recordTraceStage(
    VideoProfileStage stage,
    SteadyClock::time_point start_time,
    std::chrono::microseconds duration,
    std::optional<std::int64_t> frame_timestamp_us)
  {
    if (trace_recorder == nullptr) {
      return;
    }
    std::vector<TraceArg> args;
    if (frame_timestamp_us.has_value()) {
      addTraceArg(args, "frame_timestamp_us", *frame_timestamp_us);
    }
    trace_recorder->recordComplete(
      spec.stream_key, videoProfileStageToString(stage), start_time, duration, std::move(args));
  }

  VideoStreamSpec spec;
  std::mutex mutex;
  std::shared_ptr<VideoTraceRecorder> trace_recorder;
  bool activity_since_last_summary = false;

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

  std::optional<SteadyClock::time_point> last_ingress_arrival;
  std::optional<SteadyClock::time_point> last_output_arrival;
  std::optional<std::int64_t> last_source_timestamp_us;
  std::deque<std::pair<std::int64_t, SteadyClock::time_point>> pending_source_frames;

  MetricSamples ingress_arrival_gap_ms;
  MetricSamples output_arrival_gap_ms;
  MetricSamples source_timestamp_gap_ms;
  MetricSamples appsrc_to_sample_ms;
  MetricSamples source_to_output_submit_ms;
  MetricSamples push_to_appsrc_ms;
  MetricSamples sample_callback_ms;
  MetricSamples sample_unpack_ms;
  MetricSamples frame_sink_ms;
  MetricSamples publisher_handle_frame_ms;
  MetricSamples ensure_track_ms;
  MetricSamples capture_frame_ms;
};

struct VideoProfilingRegistry::Impl
{
  explicit Impl(rclcpp::Logger profiling_logger, VideoProfilingConfig profiling_config)
  : logger(std::move(profiling_logger))
  , config(std::move(profiling_config))
  , recorder(config.enabled ? std::make_shared<VideoTraceRecorder>(config) : nullptr)
  {}

  rclcpp::Logger logger;
  VideoProfilingConfig config;
  std::shared_ptr<VideoTraceRecorder> recorder;
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<VideoStreamProfiler>> profilers;
  std::size_t last_reported_trace_drop_count = 0;
};

const char * videoProfileStageToString(VideoProfileStage stage)
{
  switch (stage) {
    case VideoProfileStage::kPushToAppSrc:
      return "source.queue_to_gst_ms";
    case VideoProfileStage::kSampleCallback:
      return "gst.sample_total_ms";
    case VideoProfileStage::kSampleUnpack:
      return "gst.repack_i420_ms";
    case VideoProfileStage::kFrameSink:
      return "bridge.to_publisher_ms";
    case VideoProfileStage::kPublisherHandleFrame:
      return "publish.total_ms";
    case VideoProfileStage::kEnsureTrack:
      return "publish.ensure_track_ms";
    case VideoProfileStage::kCaptureFrame:
      return "livekit.submit_ms";
  }

  return "unknown";
}

bool VideoStreamProfileSummary::hasActivity() const
{
  return frames_in > 0 || frames_sampled > 0 || frames_captured > 0 || pipeline_start_count > 0 ||
         pipeline_failure_count > 0 || restart_failed_count > 0 || push_failed_count > 0 ||
         sample_unpack_failed_count > 0 || capture_failed_count > 0 || track_publish_count > 0 ||
         track_republish_count > 0 || track_unpublish_count > 0 || source_timestamp_regression_count > 0 ||
         ingress_arrival_gap_ms.hasSamples() || output_arrival_gap_ms.hasSamples() ||
         source_timestamp_gap_ms.hasSamples() || appsrc_to_sample_ms.hasSamples() ||
         source_to_output_submit_ms.hasSamples() || push_to_appsrc_ms.hasSamples() || sample_callback_ms.hasSamples() ||
         sample_unpack_ms.hasSamples() || frame_sink_ms.hasSamples() || publisher_handle_frame_ms.hasSamples() ||
         ensure_track_ms.hasSamples() || capture_frame_ms.hasSamples();
}

VideoStreamProfiler::ScopedStageTimer::ScopedStageTimer(
  VideoStreamProfiler * profiler, VideoProfileStage stage, std::optional<std::int64_t> frame_timestamp_us)
: profiler_(profiler)
, stage_(stage)
, frame_timestamp_us_(frame_timestamp_us)
, start_time_(SteadyClock::now())
{}

VideoStreamProfiler::ScopedStageTimer::~ScopedStageTimer()
{
  if (profiler_ == nullptr) {
    return;
  }
  const auto end_time = SteadyClock::now();
  profiler_->recordStageDuration(
    stage_,
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_),
    frame_timestamp_us_,
    start_time_);
}

VideoStreamProfiler::ScopedStageTimer::ScopedStageTimer(ScopedStageTimer && other) noexcept
: profiler_(other.profiler_)
, stage_(other.stage_)
, frame_timestamp_us_(other.frame_timestamp_us_)
, start_time_(other.start_time_)
{
  other.profiler_ = nullptr;
}

VideoStreamProfiler::ScopedStageTimer & VideoStreamProfiler::ScopedStageTimer::operator=(
  ScopedStageTimer && other) noexcept
{
  if (this == &other) {
    return *this;
  }
  profiler_ = other.profiler_;
  stage_ = other.stage_;
  frame_timestamp_us_ = other.frame_timestamp_us_;
  start_time_ = other.start_time_;
  other.profiler_ = nullptr;
  return *this;
}

VideoStreamProfiler::VideoStreamProfiler(VideoStreamSpec spec)
: impl_(std::make_unique<Impl>(std::move(spec)))
{}

VideoStreamProfiler::~VideoStreamProfiler() = default;

void VideoStreamProfiler::noteIngressFrame(
  SteadyClock::time_point arrival_time, std::optional<std::int64_t> source_timestamp_us)
{
  std::vector<TraceArg> args;
  if (source_timestamp_us.has_value()) {
    addTraceArg(args, "frame_timestamp_us", *source_timestamp_us);
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->frames_in++;
  impl_->recordTraceInstant(ingressTraceEventName(), std::move(args));

  if (impl_->last_ingress_arrival.has_value()) {
    const auto gap_us =
      std::chrono::duration_cast<std::chrono::microseconds>(arrival_time - *impl_->last_ingress_arrival);
    impl_->ingress_arrival_gap_ms.addMilliseconds(static_cast<double>(gap_us.count()) / 1000.0);
  }
  impl_->last_ingress_arrival = arrival_time;

  const auto validated_timestamp_us = validateSourceTimestampUs(source_timestamp_us);
  if (!validated_timestamp_us.has_value()) {
    impl_->source_timestamp_regression_count++;
    impl_->last_source_timestamp_us.reset();
    return;
  }

  if (impl_->last_source_timestamp_us.has_value()) {
    if (*validated_timestamp_us < *impl_->last_source_timestamp_us) {
      impl_->source_timestamp_regression_count++;
      impl_->pending_source_frames.clear();
      impl_->last_source_timestamp_us = validated_timestamp_us;
      return;
    }
    impl_->source_timestamp_gap_ms.addMilliseconds(
      static_cast<double>(*validated_timestamp_us - *impl_->last_source_timestamp_us) / 1000.0);
  }
  impl_->pending_source_frames.emplace_back(*validated_timestamp_us, arrival_time);
  while (impl_->pending_source_frames.size() > 1024U) {
    impl_->pending_source_frames.pop_front();
  }
  impl_->last_source_timestamp_us = validated_timestamp_us;
}

void VideoStreamProfiler::noteSampledFrame(std::optional<std::int64_t> frame_timestamp_us)
{
  const auto sample_ready_time = SteadyClock::now();
  std::vector<TraceArg> args;
  if (frame_timestamp_us.has_value()) {
    addTraceArg(args, "frame_timestamp_us", *frame_timestamp_us);
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->frames_sampled++;
  impl_->recordTraceInstant(sampledTraceEventName(), std::move(args));

  const auto validated_timestamp_us = validateSourceTimestampUs(frame_timestamp_us);
  if (!validated_timestamp_us.has_value()) {
    return;
  }

  while (!impl_->pending_source_frames.empty() && impl_->pending_source_frames.front().first < *validated_timestamp_us)
  {
    impl_->pending_source_frames.pop_front();
  }

  if (!impl_->pending_source_frames.empty() && impl_->pending_source_frames.front().first == *validated_timestamp_us) {
    const auto source_to_sample_ready = std::chrono::duration_cast<std::chrono::microseconds>(
      sample_ready_time - impl_->pending_source_frames.front().second);
    impl_->appsrc_to_sample_ms.addMilliseconds(static_cast<double>(source_to_sample_ready.count()) / 1000.0);
    std::vector<TraceArg> span_args;
    addTraceArg(span_args, "frame_timestamp_us", *validated_timestamp_us);
    impl_->recordTraceComplete(
      sourceToSampleReadyTraceEventName(),
      impl_->pending_source_frames.front().second,
      source_to_sample_ready,
      std::move(span_args));
  }
}

void VideoStreamProfiler::noteFrameCaptured(std::optional<std::int64_t> frame_timestamp_us)
{
  const auto capture_complete_time = SteadyClock::now();
  std::vector<TraceArg> args;
  if (frame_timestamp_us.has_value()) {
    addTraceArg(args, "frame_timestamp_us", *frame_timestamp_us);
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->frames_captured++;
  impl_->recordTraceInstant(capturedTraceEventName(), std::move(args));

  if (impl_->last_output_arrival.has_value()) {
    const auto gap_us =
      std::chrono::duration_cast<std::chrono::microseconds>(capture_complete_time - *impl_->last_output_arrival);
    impl_->output_arrival_gap_ms.addMilliseconds(static_cast<double>(gap_us.count()) / 1000.0);
  }
  impl_->last_output_arrival = capture_complete_time;

  const auto validated_timestamp_us = validateSourceTimestampUs(frame_timestamp_us);
  if (!validated_timestamp_us.has_value()) {
    return;
  }

  while (!impl_->pending_source_frames.empty() && impl_->pending_source_frames.front().first < *validated_timestamp_us)
  {
    impl_->pending_source_frames.pop_front();
  }

  if (!impl_->pending_source_frames.empty() && impl_->pending_source_frames.front().first == *validated_timestamp_us) {
    const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
      capture_complete_time - impl_->pending_source_frames.front().second);
    impl_->source_to_output_submit_ms.addMilliseconds(static_cast<double>(latency_us.count()) / 1000.0);
    std::vector<TraceArg> span_args;
    addTraceArg(span_args, "frame_timestamp_us", *validated_timestamp_us);
    impl_->recordTraceComplete(
      sourceToLiveKitSubmitTraceEventName(),
      impl_->pending_source_frames.front().second,
      latency_us,
      std::move(span_args));
    impl_->pending_source_frames.pop_front();
  }
}

void VideoStreamProfiler::notePipelineStart()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->pipeline_start_count++;
  impl_->recordTraceInstant(pipelineStartTraceEventName());
}

void VideoStreamProfiler::notePipelineFailure(const std::string & reason)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "reason", reason);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->pipeline_failure_count++;
  impl_->recordTraceInstant(pipelineFailureTraceEventName(), std::move(args));
}

void VideoStreamProfiler::noteRestartFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->restart_failed_count++;
  impl_->recordTraceInstant(restartFailureTraceEventName(), std::move(args));
}

void VideoStreamProfiler::notePushFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->push_failed_count++;
  impl_->recordTraceInstant(pushFailureTraceEventName(), std::move(args));
}

void VideoStreamProfiler::noteSampleUnpackFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->sample_unpack_failed_count++;
  impl_->recordTraceInstant(sampleUnpackFailureTraceEventName(), std::move(args));
}

void VideoStreamProfiler::noteCaptureFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->capture_failed_count++;
  impl_->recordTraceInstant(captureFailureTraceEventName(), std::move(args));
}

void VideoStreamProfiler::noteTrackPublished(int width, int height, bool republished)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "width", static_cast<std::int64_t>(width));
  addTraceArg(args, "height", static_cast<std::int64_t>(height));
  addTraceArg(args, "republished", republished);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  if (republished) {
    impl_->track_republish_count++;
  } else {
    impl_->track_publish_count++;
  }
  impl_->recordTraceInstant(trackPublishedTraceEventName(republished), std::move(args));
}

void VideoStreamProfiler::noteTrackUnpublishing()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  impl_->track_unpublish_count++;
  impl_->recordTraceInstant(trackUnpublishingTraceEventName());
}

void VideoStreamProfiler::recordStageDuration(
  VideoProfileStage stage,
  std::chrono::microseconds duration,
  std::optional<std::int64_t> frame_timestamp_us,
  std::optional<SteadyClock::time_point> start_time)
{
  const double duration_ms = static_cast<double>(duration.count()) / 1000.0;

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->markActivityLocked();
  switch (stage) {
    case VideoProfileStage::kPushToAppSrc:
      impl_->push_to_appsrc_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kSampleCallback:
      impl_->sample_callback_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kSampleUnpack:
      impl_->sample_unpack_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kFrameSink:
      impl_->frame_sink_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kPublisherHandleFrame:
      impl_->publisher_handle_frame_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kEnsureTrack:
      impl_->ensure_track_ms.addMilliseconds(duration_ms);
      break;
    case VideoProfileStage::kCaptureFrame:
      impl_->capture_frame_ms.addMilliseconds(duration_ms);
      break;
  }

  impl_->recordTraceStage(
    stage, start_time.has_value() ? *start_time : SteadyClock::now() - duration, duration, frame_timestamp_us);
}

std::optional<VideoStreamProfileSummary> VideoStreamProfiler::collectAndResetSummary()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->activity_since_last_summary) {
    return std::nullopt;
  }

  VideoStreamProfileSummary summary;
  summary.stream_key = impl_->spec.stream_key;
  summary.track_name = impl_->spec.track_name;
  summary.input_kind = videoInputKindToString(impl_->spec.input_kind);
  summary.ingest_mode = impl_->spec.ingest_mode;
  summary.frames_in = impl_->frames_in;
  summary.frames_sampled = impl_->frames_sampled;
  summary.frames_captured = impl_->frames_captured;
  summary.pipeline_start_count = impl_->pipeline_start_count;
  summary.pipeline_failure_count = impl_->pipeline_failure_count;
  summary.restart_failed_count = impl_->restart_failed_count;
  summary.push_failed_count = impl_->push_failed_count;
  summary.sample_unpack_failed_count = impl_->sample_unpack_failed_count;
  summary.capture_failed_count = impl_->capture_failed_count;
  summary.track_publish_count = impl_->track_publish_count;
  summary.track_republish_count = impl_->track_republish_count;
  summary.track_unpublish_count = impl_->track_unpublish_count;
  summary.source_timestamp_regression_count = impl_->source_timestamp_regression_count;
  summary.ingress_arrival_gap_ms = summarizeAndResetMetric(impl_->ingress_arrival_gap_ms);
  summary.output_arrival_gap_ms = summarizeAndResetMetric(impl_->output_arrival_gap_ms);
  summary.source_timestamp_gap_ms = summarizeAndResetMetric(impl_->source_timestamp_gap_ms);
  summary.appsrc_to_sample_ms = summarizeAndResetMetric(impl_->appsrc_to_sample_ms);
  summary.source_to_output_submit_ms = summarizeAndResetMetric(impl_->source_to_output_submit_ms);
  summary.push_to_appsrc_ms = summarizeAndResetMetric(impl_->push_to_appsrc_ms);
  summary.sample_callback_ms = summarizeAndResetMetric(impl_->sample_callback_ms);
  summary.sample_unpack_ms = summarizeAndResetMetric(impl_->sample_unpack_ms);
  summary.frame_sink_ms = summarizeAndResetMetric(impl_->frame_sink_ms);
  summary.publisher_handle_frame_ms = summarizeAndResetMetric(impl_->publisher_handle_frame_ms);
  summary.ensure_track_ms = summarizeAndResetMetric(impl_->ensure_track_ms);
  summary.capture_frame_ms = summarizeAndResetMetric(impl_->capture_frame_ms);

  impl_->activity_since_last_summary = false;
  impl_->frames_in = 0;
  impl_->frames_sampled = 0;
  impl_->frames_captured = 0;
  impl_->pipeline_start_count = 0;
  impl_->pipeline_failure_count = 0;
  impl_->restart_failed_count = 0;
  impl_->push_failed_count = 0;
  impl_->sample_unpack_failed_count = 0;
  impl_->capture_failed_count = 0;
  impl_->track_publish_count = 0;
  impl_->track_republish_count = 0;
  impl_->track_unpublish_count = 0;
  impl_->source_timestamp_regression_count = 0;
  return summary;
}

VideoProfilingRegistry::VideoProfilingRegistry(rclcpp::Logger logger, VideoProfilingConfig config)
: impl_(std::make_unique<Impl>(std::move(logger), std::move(config)))
{}

VideoProfilingRegistry::~VideoProfilingRegistry() = default;

bool VideoProfilingRegistry::enabled() const
{
  return impl_->config.enabled;
}

const VideoProfilingConfig & VideoProfilingRegistry::config() const
{
  return impl_->config;
}

std::shared_ptr<VideoStreamProfiler> VideoProfilingRegistry::getOrCreateStreamProfiler(const VideoStreamSpec & spec)
{
  if (!enabled()) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto [it, inserted] = impl_->profilers.try_emplace(spec.stream_key);
  if (inserted || it->second == nullptr) {
    auto profiler = std::make_shared<VideoStreamProfiler>(spec);
    profiler->impl_->trace_recorder = impl_->recorder;
    if (impl_->recorder != nullptr) {
      std::vector<TraceArg> args;
      addTraceArg(args, "track_name", spec.track_name);
      addTraceArg(args, "input_kind", std::string(videoInputKindToString(spec.input_kind)));
      addTraceArg(args, "ingest_mode", spec.ingest_mode);
      impl_->recorder->recordInstant(
        spec.stream_key, streamRegisteredTraceEventName(), SteadyClock::now(), std::move(args));
    }
    it->second = std::move(profiler);
  }
  return it->second;
}

std::vector<VideoStreamProfileSummary> VideoProfilingRegistry::collectAndResetSummaries()
{
  std::vector<std::shared_ptr<VideoStreamProfiler>> profilers;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    profilers.reserve(impl_->profilers.size());
    for (const auto & entry : impl_->profilers) {
      if (entry.second != nullptr) {
        profilers.push_back(entry.second);
      }
    }
  }

  std::vector<VideoStreamProfileSummary> summaries;
  summaries.reserve(profilers.size());
  for (const auto & profiler : profilers) {
    if (const auto summary = profiler->collectAndResetSummary(); summary.has_value()) {
      summaries.push_back(*summary);
    }
  }
  return summaries;
}

std::size_t VideoProfilingRegistry::traceDroppedEventCount() const
{
  return impl_->recorder == nullptr ? 0U : impl_->recorder->droppedEventCount();
}

void VideoProfilingRegistry::emitEnabledLog() const
{
  if (!enabled()) {
    return;
  }

  LogEvent(impl_->logger, "video_profiling_enabled")
    .field("summary_interval_ms", impl_->config.summary_interval.count())
    .field("trace_file", impl_->config.trace_file)
    .field("trace_max_events", impl_->config.trace_max_events)
    .info();
}

void VideoProfilingRegistry::emitSummaryLogs()
{
  if (!enabled()) {
    return;
  }

  const auto summaries = collectAndResetSummaries();
  for (const auto & summary : summaries) {
    LogEvent event(impl_->logger, "video_profile_summary");
    event.field("stream_key", summary.stream_key)
      .field("track_name", summary.track_name)
      .field("input_kind", summary.input_kind)
      .field("ingest_mode", summary.ingest_mode)
      .field(summaryFramesInFieldName(), summary.frames_in)
      .field(summaryFramesSampledFieldName(), summary.frames_sampled)
      .field(summaryFramesCapturedFieldName(), summary.frames_captured)
      .field(summaryPipelineStartCountFieldName(), summary.pipeline_start_count)
      .field(summaryPipelineFailureCountFieldName(), summary.pipeline_failure_count)
      .field(summaryRestartFailedCountFieldName(), summary.restart_failed_count)
      .field(summaryPushFailedCountFieldName(), summary.push_failed_count)
      .field(summarySampleUnpackFailedCountFieldName(), summary.sample_unpack_failed_count)
      .field(summaryCaptureFailedCountFieldName(), summary.capture_failed_count)
      .field(summaryTrackPublishCountFieldName(), summary.track_publish_count)
      .field(summaryTrackRepublishCountFieldName(), summary.track_republish_count)
      .field(summaryTrackUnpublishCountFieldName(), summary.track_unpublish_count)
      .field(summarySourceTimestampRegressionCountFieldName(), summary.source_timestamp_regression_count);
    addMetricFields(event, summaryIngressArrivalGapFieldName(), summary.ingress_arrival_gap_ms);
    addMetricFields(event, summaryOutputArrivalGapFieldName(), summary.output_arrival_gap_ms);
    addMetricFields(event, summarySourceTimestampGapFieldName(), summary.source_timestamp_gap_ms);
    addMetricFields(event, summarySourceToSampleReadyFieldName(), summary.appsrc_to_sample_ms);
    addMetricFields(event, summarySourceToOutputSubmitFieldName(), summary.source_to_output_submit_ms);
    addMetricFields(event, summaryPushToAppSrcFieldName(), summary.push_to_appsrc_ms);
    addMetricFields(event, summarySampleCallbackFieldName(), summary.sample_callback_ms);
    addMetricFields(event, summarySampleUnpackFieldName(), summary.sample_unpack_ms);
    addMetricFields(event, summaryFrameSinkFieldName(), summary.frame_sink_ms);
    addMetricFields(event, summaryPublisherHandleFieldName(), summary.publisher_handle_frame_ms);
    addMetricFields(event, summaryEnsureTrackFieldName(), summary.ensure_track_ms);
    addMetricFields(event, summaryCaptureFrameFieldName(), summary.capture_frame_ms);
    event.info();
  }

  const std::size_t trace_drop_count = traceDroppedEventCount();
  const std::size_t trace_drop_delta = trace_drop_count - impl_->last_reported_trace_drop_count;
  if (!summaries.empty() || trace_drop_delta > 0U) {
    LogEvent(impl_->logger, "video_profile_summary")
      .field("stream_key", "<aggregate>")
      .field("track_name", "<aggregate>")
      .field("input_kind", "<aggregate>")
      .field("ingest_mode", "<aggregate>")
      .field("active_stream_count", summaries.size())
      .field("trace_dropped_event_count", trace_drop_delta)
      .info();
  }
  impl_->last_reported_trace_drop_count = trace_drop_count;
}

void VideoProfilingRegistry::flushTrace()
{
  if (!enabled() || impl_->recorder == nullptr) {
    return;
  }

  try {
    const std::size_t event_count = impl_->recorder->writeTraceFile();
    LogEvent(impl_->logger, "video_profile_trace_flushed")
      .field("trace_file", impl_->config.trace_file)
      .field("event_count", event_count)
      .field("dropped_event_count", impl_->recorder->droppedEventCount())
      .info();
  } catch (const std::exception & error) {
    LogEvent(impl_->logger, "video_profile_trace_flush_failed")
      .field("trace_file", impl_->config.trace_file)
      .field("error", error.what())
      .error();
  }
}

}  // namespace livekit_ros2_bridge
