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
#include <array>
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
#include <stdexcept>
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
using TraceValue = std::variant<std::int64_t, double, bool, std::string>;

enum class TraceKind
{
  kComplete,
  kInstant,
};

struct TraceArg
{
  std::string key;
  TraceValue value;
};

struct TraceEvent
{
  TraceKind kind = TraceKind::kInstant;
  std::string category;
  std::string name;
  std::int64_t timestamp_us = 0;
  std::int64_t duration_us = 0;
  std::uint64_t thread_id = 0;
  std::vector<TraceArg> args;
};

struct DurationSamples
{
  std::vector<double> durations_ms;

  void addDurationMs(double duration_ms)
  {
    durations_ms.push_back(duration_ms);
  }

  VideoProfileDurationSummary takeSummary()
  {
    VideoProfileDurationSummary summary;
    summary.sample_count = durations_ms.size();
    if (durations_ms.empty()) {
      return summary;
    }

    const double sum_ms = std::accumulate(durations_ms.begin(), durations_ms.end(), 0.0);
    summary.avg_ms = sum_ms / static_cast<double>(durations_ms.size());

    std::sort(durations_ms.begin(), durations_ms.end());
    const std::size_t p95_index =
      static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(durations_ms.size()))) - 1U;
    summary.p95_ms = durations_ms[std::min(p95_index, durations_ms.size() - 1U)];
    summary.max_ms = durations_ms.back();
    durations_ms.clear();
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

nlohmann::json traceArgToJson(const TraceValue & value)
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

void addCountFieldIfNonZero(LogEvent & event, const char * field_name, std::size_t count)
{
  if (count == 0U) {
    return;
  }

  event.field(field_name, count);
}

void addProfilerIdentityFields(LogEvent & event, const VideoStreamSpec & spec)
{
  event.field("stream_key", spec.stream_key)
    .field("track_name", spec.track_name)
    .field("input_kind", videoInputKindToString(spec.input_kind))
    .field("ingest_mode", spec.ingest_mode);
}

void addProfilerIdentityFields(LogEvent & event, const VideoStreamProfileSummary & summary)
{
  event.field("stream_key", summary.stream_key)
    .field("track_name", summary.track_name)
    .field("input_kind", summary.input_kind)
    .field("ingest_mode", summary.ingest_mode);
}

bool hasProfilerIdentityMismatch(const VideoStreamSpec & active_spec, const VideoStreamSpec & requested_spec)
{
  return active_spec.track_name != requested_spec.track_name || active_spec.input_kind != requested_spec.input_kind ||
         active_spec.ingest_mode != requested_spec.ingest_mode;
}

void addProfilerIdentityMismatchFields(
  LogEvent & event, const VideoStreamSpec & active_spec, const VideoStreamSpec & requested_spec)
{
  if (active_spec.track_name != requested_spec.track_name) {
    event.field("active_track_name", active_spec.track_name).field("requested_track_name", requested_spec.track_name);
  }

  if (active_spec.input_kind != requested_spec.input_kind) {
    event.field("active_input_kind", videoInputKindToString(active_spec.input_kind))
      .field("requested_input_kind", videoInputKindToString(requested_spec.input_kind));
  }

  if (active_spec.ingest_mode != requested_spec.ingest_mode) {
    event.field("active_ingest_mode", active_spec.ingest_mode)
      .field("requested_ingest_mode", requested_spec.ingest_mode);
  }
}

constexpr char kIngressTraceEventName[] = "source.received";
constexpr char kSampledTraceEventName[] = "gst.sample_ready";
constexpr char kCapturedTraceEventName[] = "livekit.frame_submitted";
constexpr char kPipelineStartTraceEventName[] = "gst.pipeline_started";
constexpr char kPipelineFailureTraceEventName[] = "gst.pipeline_failed";
constexpr char kRestartFailureTraceEventName[] = "gst.restart_failed";
constexpr char kPushFailureTraceEventName[] = "source.queue_to_gst_failed";
constexpr char kSampleUnpackFailureTraceEventName[] = "gst.repack_failed";
constexpr char kCaptureFailureTraceEventName[] = "livekit.submit_failed";
constexpr char kSourceToSampleReadyFieldName[] = "source_to_sample_ready_ms";
constexpr char kSourceToLiveKitSubmitFieldName[] = "source_to_livekit_submit_ms";
constexpr char kTrackPublishedTraceEventName[] = "publish.track_published";
constexpr char kTrackRepublishedTraceEventName[] = "publish.track_republished";
constexpr char kTrackUnpublishingTraceEventName[] = "publish.track_unpublishing";
constexpr char kStreamRegisteredTraceEventName[] = "stream.registered";

constexpr char kFramesInCountFieldName[] = "source.received_count";
constexpr char kFramesSampledCountFieldName[] = "gst.sample_count";
constexpr char kFramesCapturedCountFieldName[] = "livekit.frame_submitted_count";
constexpr char kPipelineStartCountFieldName[] = "gst.pipeline_started_count";
constexpr char kPipelineFailureCountFieldName[] = "gst.pipeline_failed_count";
constexpr char kRestartFailedCountFieldName[] = "gst.restart_failed_count";
constexpr char kPushFailedCountFieldName[] = "source.queue_to_gst_failed_count";
constexpr char kSampleUnpackFailedCountFieldName[] = "gst.repack_failed_count";
constexpr char kCaptureFailedCountFieldName[] = "livekit.submit_failed_count";
constexpr char kTrackPublishCountFieldName[] = "publish.track_published_count";
constexpr char kTrackRepublishCountFieldName[] = "publish.track_republished_count";
constexpr char kTrackUnpublishCountFieldName[] = "publish.track_unpublishing_count";
constexpr char kSourceTimestampRegressionCountFieldName[] = "source.timestamp_regression_count";

constexpr char kIngressArrivalGapFieldName[] = "source.interarrival_gap_ms";
constexpr char kSourceTimestampGapFieldName[] = "source.timestamp_gap_ms";
constexpr char kOutputArrivalGapFieldName[] = "livekit.interarrival_gap_ms";
constexpr char kPushToAppSrcStageName[] = "source.queue_to_gst_ms";
constexpr char kSampleCallbackStageName[] = "gst.sample_total_ms";
constexpr char kSampleUnpackStageName[] = "gst.repack_i420_ms";
constexpr char kFrameSinkStageName[] = "bridge.to_publisher_ms";
constexpr char kPublisherHandleStageName[] = "publish.total_ms";
constexpr char kEnsureTrackStageName[] = "publish.ensure_track_ms";
constexpr char kCaptureFrameStageName[] = "livekit.submit_ms";

// Emits Chrome Trace Event JSON using steady-clock-relative timestamps so trace
// durations stay monotonic even if the wall clock changes. Wall-clock metadata
// is recorded separately for offline correlation with logs and external events.
class TraceRecorder final
{
public:
  explicit TraceRecorder(const VideoProfilingConfig & config)
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
    event.kind = TraceKind::kComplete;
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
    event.kind = TraceKind::kInstant;
    event.category = std::move(category);
    event.name = std::move(name);
    event.timestamp_us = toTraceTimestampUs(event_time);
    event.thread_id = currentThreadId();
    event.args = std::move(args);
    appendEvent(std::move(event));
  }

  std::size_t droppedEvents() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_event_count_;
  }

  std::size_t writeTrace() const
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
      if (event.kind == TraceKind::kComplete) {
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

// Stores a bounded ingress window for timestamp-based correlation across
// sample and capture callbacks.
class IngressMatcher final
{
public:
  enum class RecordResult
  {
    kAccepted,
    kTimestampRegression,
  };

  enum class MatchMode
  {
    kKeep,
    kConsume,
  };

  void reset()
  {
    last_timestamp_us_.reset();
  }

  RecordResult record(std::int64_t timestamp_us, SteadyClock::time_point arrival_time, DurationSamples & gap_ms)
  {
    if (last_timestamp_us_.has_value()) {
      if (timestamp_us < *last_timestamp_us_) {
        pending_frames_.clear();
        last_timestamp_us_ = timestamp_us;
        return RecordResult::kTimestampRegression;
      }

      gap_ms.addDurationMs(static_cast<double>(timestamp_us - *last_timestamp_us_) / 1000.0);
    }

    pending_frames_.emplace_back(timestamp_us, arrival_time);
    while (pending_frames_.size() > kMaxPendingFrames) {
      pending_frames_.pop_front();
    }

    last_timestamp_us_ = timestamp_us;
    return RecordResult::kAccepted;
  }

  std::optional<SteadyClock::time_point> match(std::int64_t timestamp_us, MatchMode mode)
  {
    // Samples and captures are expected to observe source timestamps in order.
    // Once a newer timestamp is requested, older unmatched ingress frames can
    // no longer be correlated safely and are dropped from the front.
    while (!pending_frames_.empty() && pending_frames_.front().first < timestamp_us) {
      pending_frames_.pop_front();
    }

    if (pending_frames_.empty() || pending_frames_.front().first != timestamp_us) {
      return std::nullopt;
    }

    const auto ingress_time = pending_frames_.front().second;
    if (mode == MatchMode::kConsume) {
      pending_frames_.pop_front();
    }
    return ingress_time;
  }

private:
  static constexpr std::size_t kMaxPendingFrames = 1024U;

  std::optional<std::int64_t> last_timestamp_us_;
  std::deque<std::pair<std::int64_t, SteadyClock::time_point>> pending_frames_;
};

}  // namespace

// Protected by mutex. Ingress correlation state survives takeSummary() so
// frames crossing a summary boundary can still be matched end-to-end.
struct VideoStreamProfiler::Impl
{
  explicit Impl(VideoStreamSpec spec)
  : spec(std::move(spec))
  {}

  VideoStreamSpec spec;
  std::mutex mutex;
  std::shared_ptr<TraceRecorder> recorder;
  bool has_activity = false;

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
  IngressMatcher ingress_matcher;

  DurationSamples ingress_arrival_gap_ms;
  DurationSamples output_arrival_gap_ms;
  DurationSamples source_timestamp_gap_ms;
  DurationSamples appsrc_to_sample_ms;
  DurationSamples source_to_output_submit_ms;
  DurationSamples push_to_appsrc_ms;
  DurationSamples sample_callback_ms;
  DurationSamples sample_unpack_ms;
  DurationSamples frame_sink_ms;
  DurationSamples publisher_handle_frame_ms;
  DurationSamples ensure_track_ms;
  DurationSamples capture_frame_ms;
};

struct VideoProfilingRegistry::Impl
{
  explicit Impl(rclcpp::Logger logger, VideoProfilingConfig config)
  : logger(std::move(logger))
  , config(std::move(config))
  , recorder(this->config.enabled ? std::make_shared<TraceRecorder>(this->config) : nullptr)
  {}

  rclcpp::Logger logger;
  VideoProfilingConfig config;
  std::shared_ptr<TraceRecorder> recorder;
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<VideoStreamProfiler>> profilers;
  std::size_t last_logged_drop_count = 0;
};

namespace
{

struct StageMetric
{
  VideoProfileStage stage;
  const char * metric_name = "unknown";
  DurationSamples VideoStreamProfiler::Impl::* samples = nullptr;
  VideoProfileDurationSummary VideoStreamProfileSummary::* summary = nullptr;
};

const std::array<StageMetric, 7> kStageMetrics{{
  {VideoProfileStage::kPushToAppSrc,
   kPushToAppSrcStageName,
   &VideoStreamProfiler::Impl::push_to_appsrc_ms,
   &VideoStreamProfileSummary::push_to_appsrc_ms},
  {VideoProfileStage::kSampleCallback,
   kSampleCallbackStageName,
   &VideoStreamProfiler::Impl::sample_callback_ms,
   &VideoStreamProfileSummary::sample_callback_ms},
  {VideoProfileStage::kSampleUnpack,
   kSampleUnpackStageName,
   &VideoStreamProfiler::Impl::sample_unpack_ms,
   &VideoStreamProfileSummary::sample_unpack_ms},
  {VideoProfileStage::kFrameSink,
   kFrameSinkStageName,
   &VideoStreamProfiler::Impl::frame_sink_ms,
   &VideoStreamProfileSummary::frame_sink_ms},
  {VideoProfileStage::kPublisherHandleFrame,
   kPublisherHandleStageName,
   &VideoStreamProfiler::Impl::publisher_handle_frame_ms,
   &VideoStreamProfileSummary::publisher_handle_frame_ms},
  {VideoProfileStage::kEnsureTrack,
   kEnsureTrackStageName,
   &VideoStreamProfiler::Impl::ensure_track_ms,
   &VideoStreamProfileSummary::ensure_track_ms},
  {VideoProfileStage::kCaptureFrame,
   kCaptureFrameStageName,
   &VideoStreamProfiler::Impl::capture_frame_ms,
   &VideoStreamProfileSummary::capture_frame_ms},
}};

const StageMetric & stageMetric(VideoProfileStage stage)
{
  const auto metric = std::find_if(
    kStageMetrics.begin(), kStageMetrics.end(), [stage](const auto & candidate) { return candidate.stage == stage; });
  if (metric == kStageMetrics.end()) {
    throw std::logic_error("unsupported video profile stage");
  }
  return *metric;
}

std::vector<TraceArg> makeFrameTimestampArgs(std::optional<std::int64_t> frame_timestamp_us)
{
  std::vector<TraceArg> args;
  if (frame_timestamp_us.has_value()) {
    addTraceArg(args, "frame_timestamp_us", *frame_timestamp_us);
  }
  return args;
}

}  // namespace

const char * videoProfileStageToString(VideoProfileStage stage)
{
  return stageMetric(stage).metric_name;
}

bool VideoStreamProfileSummary::hasActivity() const
{
  if (
    frames_in > 0 || frames_sampled > 0 || frames_captured > 0 || pipeline_start_count > 0 ||
    pipeline_failure_count > 0 || restart_failed_count > 0 || push_failed_count > 0 || sample_unpack_failed_count > 0 ||
    capture_failed_count > 0 || track_publish_count > 0 || track_republish_count > 0 || track_unpublish_count > 0 ||
    source_timestamp_regression_count > 0 || ingress_arrival_gap_ms.hasSamples() ||
    output_arrival_gap_ms.hasSamples() || source_timestamp_gap_ms.hasSamples() || appsrc_to_sample_ms.hasSamples() ||
    source_to_output_submit_ms.hasSamples())
  {
    return true;
  }

  for (const auto & metric : kStageMetrics) {
    if ((this->*(metric.summary)).hasSamples()) {
      return true;
    }
  }
  return false;
}

VideoStreamProfiler::StageTimer::StageTimer(
  VideoStreamProfiler * profiler, VideoProfileStage stage, std::optional<std::int64_t> frame_timestamp_us)
: profiler_(profiler)
, stage_(stage)
, frame_timestamp_us_(frame_timestamp_us)
, start_time_(SteadyClock::now())
{}

VideoStreamProfiler::StageTimer::~StageTimer()
{
  if (profiler_ == nullptr) {
    return;
  }
  const auto end_time = SteadyClock::now();
  profiler_->recordStage(
    stage_,
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_),
    frame_timestamp_us_,
    start_time_);
}

VideoStreamProfiler::StageTimer::StageTimer(StageTimer && other) noexcept
: profiler_(other.profiler_)
, stage_(other.stage_)
, frame_timestamp_us_(other.frame_timestamp_us_)
, start_time_(other.start_time_)
{
  other.profiler_ = nullptr;
}

VideoStreamProfiler::StageTimer & VideoStreamProfiler::StageTimer::operator=(StageTimer && other) noexcept
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

void VideoStreamProfiler::noteIngress(
  SteadyClock::time_point arrival_time, std::optional<std::int64_t> source_timestamp_us)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->frames_in++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kIngressTraceEventName, SteadyClock::now(), makeFrameTimestampArgs(source_timestamp_us));
  }

  if (impl_->last_ingress_arrival.has_value()) {
    const auto gap_us =
      std::chrono::duration_cast<std::chrono::microseconds>(arrival_time - *impl_->last_ingress_arrival);
    impl_->ingress_arrival_gap_ms.addDurationMs(static_cast<double>(gap_us.count()) / 1000.0);
  }
  impl_->last_ingress_arrival = arrival_time;

  if (!source_timestamp_us.has_value() || *source_timestamp_us < 0) {
    // Drop pending matches once source timestamps stop being usable.
    impl_->source_timestamp_regression_count++;
    impl_->ingress_matcher.reset();
    return;
  }

  if (
    impl_->ingress_matcher.record(*source_timestamp_us, arrival_time, impl_->source_timestamp_gap_ms) ==
    IngressMatcher::RecordResult::kTimestampRegression)
  {
    impl_->source_timestamp_regression_count++;
  }
}

void VideoStreamProfiler::noteSample(std::optional<std::int64_t> frame_timestamp_us)
{
  const auto sample_ready_time = SteadyClock::now();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->frames_sampled++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kSampledTraceEventName, SteadyClock::now(), makeFrameTimestampArgs(frame_timestamp_us));
  }

  if (!frame_timestamp_us.has_value() || *frame_timestamp_us < 0) {
    return;
  }

  const auto ingress_time = impl_->ingress_matcher.match(*frame_timestamp_us, IngressMatcher::MatchMode::kKeep);
  if (!ingress_time.has_value()) {
    return;
  }

  const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(sample_ready_time - *ingress_time);
  impl_->appsrc_to_sample_ms.addDurationMs(static_cast<double>(latency_us.count()) / 1000.0);

  if (impl_->recorder != nullptr) {
    impl_->recorder->recordComplete(
      impl_->spec.stream_key,
      kSourceToSampleReadyFieldName,
      *ingress_time,
      latency_us,
      makeFrameTimestampArgs(frame_timestamp_us));
  }
}

void VideoStreamProfiler::noteCapture(std::optional<std::int64_t> frame_timestamp_us)
{
  const auto capture_complete_time = SteadyClock::now();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->frames_captured++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kCapturedTraceEventName, SteadyClock::now(), makeFrameTimestampArgs(frame_timestamp_us));
  }

  if (impl_->last_output_arrival.has_value()) {
    const auto gap_us =
      std::chrono::duration_cast<std::chrono::microseconds>(capture_complete_time - *impl_->last_output_arrival);
    impl_->output_arrival_gap_ms.addDurationMs(static_cast<double>(gap_us.count()) / 1000.0);
  }
  impl_->last_output_arrival = capture_complete_time;

  if (!frame_timestamp_us.has_value() || *frame_timestamp_us < 0) {
    return;
  }

  const auto ingress_time = impl_->ingress_matcher.match(*frame_timestamp_us, IngressMatcher::MatchMode::kConsume);
  if (!ingress_time.has_value()) {
    return;
  }

  const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(capture_complete_time - *ingress_time);
  impl_->source_to_output_submit_ms.addDurationMs(static_cast<double>(latency_us.count()) / 1000.0);

  if (impl_->recorder != nullptr) {
    impl_->recorder->recordComplete(
      impl_->spec.stream_key,
      kSourceToLiveKitSubmitFieldName,
      *ingress_time,
      latency_us,
      makeFrameTimestampArgs(frame_timestamp_us));
  }
}

void VideoStreamProfiler::notePipelineStart()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->pipeline_start_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(impl_->spec.stream_key, kPipelineStartTraceEventName, SteadyClock::now());
  }
}

void VideoStreamProfiler::notePipelineFailure(const std::string & reason)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "reason", reason);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->pipeline_failure_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kPipelineFailureTraceEventName, SteadyClock::now(), std::move(args));
  }
}

void VideoStreamProfiler::noteRestartFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->restart_failed_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kRestartFailureTraceEventName, SteadyClock::now(), std::move(args));
  }
}

void VideoStreamProfiler::notePushFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->push_failed_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kPushFailureTraceEventName, SteadyClock::now(), std::move(args));
  }
}

void VideoStreamProfiler::noteSampleUnpackFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->sample_unpack_failed_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kSampleUnpackFailureTraceEventName, SteadyClock::now(), std::move(args));
  }
}

void VideoStreamProfiler::noteCaptureFailed(const std::string & error)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "error", error);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->capture_failed_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key, kCaptureFailureTraceEventName, SteadyClock::now(), std::move(args));
  }
}

void VideoStreamProfiler::noteTrackPublished(int width, int height, bool republished)
{
  std::vector<TraceArg> args;
  addTraceArg(args, "width", static_cast<std::int64_t>(width));
  addTraceArg(args, "height", static_cast<std::int64_t>(height));
  addTraceArg(args, "republished", republished);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  if (republished) {
    impl_->track_republish_count++;
  } else {
    impl_->track_publish_count++;
  }
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(
      impl_->spec.stream_key,
      republished ? kTrackRepublishedTraceEventName : kTrackPublishedTraceEventName,
      SteadyClock::now(),
      std::move(args));
  }
}

void VideoStreamProfiler::noteTrackUnpublish()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  impl_->track_unpublish_count++;
  if (impl_->recorder != nullptr) {
    impl_->recorder->recordInstant(impl_->spec.stream_key, kTrackUnpublishingTraceEventName, SteadyClock::now());
  }
}

void VideoStreamProfiler::recordStage(
  VideoProfileStage stage,
  std::chrono::microseconds duration,
  std::optional<std::int64_t> frame_timestamp_us,
  std::optional<SteadyClock::time_point> start_time)
{
  const double duration_ms = static_cast<double>(duration.count()) / 1000.0;
  const auto & metric = stageMetric(stage);

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->has_activity = true;
  (impl_.get()->*(metric.samples)).addDurationMs(duration_ms);

  if (impl_->recorder == nullptr) {
    return;
  }

  auto args = makeFrameTimestampArgs(frame_timestamp_us);
  impl_->recorder->recordComplete(
    impl_->spec.stream_key,
    metric.metric_name,
    start_time.value_or(SteadyClock::now() - duration),
    duration,
    std::move(args));
}

std::optional<VideoStreamProfileSummary> VideoStreamProfiler::takeSummary()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->has_activity) {
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
  summary.ingress_arrival_gap_ms = impl_->ingress_arrival_gap_ms.takeSummary();
  summary.output_arrival_gap_ms = impl_->output_arrival_gap_ms.takeSummary();
  summary.source_timestamp_gap_ms = impl_->source_timestamp_gap_ms.takeSummary();
  summary.appsrc_to_sample_ms = impl_->appsrc_to_sample_ms.takeSummary();
  summary.source_to_output_submit_ms = impl_->source_to_output_submit_ms.takeSummary();
  for (const auto & metric : kStageMetrics) {
    summary.*(metric.summary) = (impl_.get()->*(metric.samples)).takeSummary();
  }

  impl_->has_activity = false;
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

std::shared_ptr<VideoStreamProfiler> VideoProfilingRegistry::getOrCreateProfiler(const VideoStreamSpec & spec)
{
  if (!enabled()) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto [it, inserted] = impl_->profilers.try_emplace(spec.stream_key);
  if (inserted || it->second == nullptr) {
    auto profiler = std::make_shared<VideoStreamProfiler>(spec);
    profiler->impl_->recorder = impl_->recorder;
    LogEvent event(impl_->logger, "video_profile_stream_registered");
    addProfilerIdentityFields(event, spec);
    event.info();
    if (impl_->recorder != nullptr) {
      std::vector<TraceArg> args;
      addTraceArg(args, "track_name", spec.track_name);
      addTraceArg(args, "input_kind", std::string(videoInputKindToString(spec.input_kind)));
      addTraceArg(args, "ingest_mode", spec.ingest_mode);
      impl_->recorder->recordInstant(
        spec.stream_key, kStreamRegisteredTraceEventName, SteadyClock::now(), std::move(args));
    }
    it->second = std::move(profiler);
  } else if (hasProfilerIdentityMismatch(it->second->impl_->spec, spec)) {
    LogEvent event(impl_->logger, "video_profile_profiler_spec_mismatch");
    event.field("stream_key", spec.stream_key);
    addProfilerIdentityMismatchFields(event, it->second->impl_->spec, spec);
    event.warn();
  }
  return it->second;
}

std::vector<VideoStreamProfileSummary> VideoProfilingRegistry::takeSummaries()
{
  std::vector<std::shared_ptr<VideoStreamProfiler>> profilers;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    // Copy the shared_ptr set under the registry lock, then release it before
    // taking each profiler's own mutex inside VideoStreamProfiler::takeSummary().
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
    if (const auto summary = profiler->takeSummary(); summary.has_value()) {
      summaries.push_back(*summary);
    }
  }
  return summaries;
}

std::size_t VideoProfilingRegistry::traceDroppedEventCount() const
{
  return impl_->recorder == nullptr ? 0U : impl_->recorder->droppedEvents();
}

void VideoProfilingRegistry::logConfig() const
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

void VideoProfilingRegistry::logSummaries()
{
  if (!enabled()) {
    return;
  }

  const auto summaries = takeSummaries();
  for (const auto & summary : summaries) {
    LogEvent event(impl_->logger, "video_profile_summary");
    addProfilerIdentityFields(event, summary);
    event.field(kFramesInCountFieldName, summary.frames_in)
      .field(kFramesSampledCountFieldName, summary.frames_sampled)
      .field(kFramesCapturedCountFieldName, summary.frames_captured);
    addCountFieldIfNonZero(event, kPipelineStartCountFieldName, summary.pipeline_start_count);
    addCountFieldIfNonZero(event, kPipelineFailureCountFieldName, summary.pipeline_failure_count);
    addCountFieldIfNonZero(event, kRestartFailedCountFieldName, summary.restart_failed_count);
    addCountFieldIfNonZero(event, kPushFailedCountFieldName, summary.push_failed_count);
    addCountFieldIfNonZero(event, kSampleUnpackFailedCountFieldName, summary.sample_unpack_failed_count);
    addCountFieldIfNonZero(event, kCaptureFailedCountFieldName, summary.capture_failed_count);
    addCountFieldIfNonZero(event, kTrackPublishCountFieldName, summary.track_publish_count);
    addCountFieldIfNonZero(event, kTrackRepublishCountFieldName, summary.track_republish_count);
    addCountFieldIfNonZero(event, kTrackUnpublishCountFieldName, summary.track_unpublish_count);
    addCountFieldIfNonZero(event, kSourceTimestampRegressionCountFieldName, summary.source_timestamp_regression_count);
    addMetricFields(event, kIngressArrivalGapFieldName, summary.ingress_arrival_gap_ms);
    addMetricFields(event, kOutputArrivalGapFieldName, summary.output_arrival_gap_ms);
    addMetricFields(event, kSourceTimestampGapFieldName, summary.source_timestamp_gap_ms);
    addMetricFields(event, kSourceToSampleReadyFieldName, summary.appsrc_to_sample_ms);
    addMetricFields(event, kSourceToLiveKitSubmitFieldName, summary.source_to_output_submit_ms);
    for (const auto & metric : kStageMetrics) {
      addMetricFields(event, metric.metric_name, summary.*(metric.summary));
    }
    event.info();
  }

  const std::size_t trace_drop_count = traceDroppedEventCount();
  const std::size_t trace_drop_delta = trace_drop_count - impl_->last_logged_drop_count;
  if (trace_drop_delta > 0U) {
    LogEvent(impl_->logger, "video_profile_trace_events_dropped").field("dropped_event_count", trace_drop_delta).warn();
  }
  impl_->last_logged_drop_count = trace_drop_count;
}

void VideoProfilingRegistry::flushTrace()
{
  if (!enabled() || impl_->recorder == nullptr) {
    return;
  }

  try {
    const std::size_t event_count = impl_->recorder->writeTrace();
    LogEvent(impl_->logger, "video_profile_trace_flushed")
      .field("trace_file", impl_->config.trace_file)
      .field("event_count", event_count)
      .field("dropped_event_count", impl_->recorder->droppedEvents())
      .info();
  } catch (const std::exception & error) {
    LogEvent(impl_->logger, "video_profile_trace_flush_failed")
      .field("trace_file", impl_->config.trace_file)
      .field("error", error.what())
      .error();
  }
}

}  // namespace livekit_ros2_bridge
