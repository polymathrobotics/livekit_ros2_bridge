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

#include "video/track_publisher.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/gstreamer_stream.hpp"
#include "video/ros_stream.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");
constexpr auto kBitrateWindow = std::chrono::seconds(3);
// Over budget means above 3/2 of the target.
constexpr std::uint64_t kOverBudgetRatioNumerator = 3U;
constexpr std::uint64_t kOverBudgetRatioDenominator = 2U;

void tryUnpublish(RoomConnection & connection, const std::shared_ptr<livekit::LocalVideoTrack> & track) noexcept
{
  if (track == nullptr) {
    return;
  }

  try {
    connection.unpublishVideoTrack(track);
  } catch (...) {
  }
}

livekit::TrackPublishOptions publishOptionsWithFrameMetadata(livekit::TrackPublishOptions options)
{
  livekit::FrameMetadataFeatures metadata_features;
  metadata_features.user_timestamp = true;
  metadata_features.frame_id = false;
  options.frame_metadata_features = metadata_features;
  return options;
}

std::optional<livekit::VideoFrameMetadata> userTimestampMetadata(std::int64_t timestamp_us)
{
  if (timestamp_us < 0) {
    return std::nullopt;
  }
  livekit::VideoFrameMetadata metadata;
  metadata.user_timestamp_us = static_cast<std::uint64_t>(timestamp_us);
  return metadata;
}

livekit::VideoCaptureOptions captureOptions(std::int64_t timestamp_us)
{
  livekit::VideoCaptureOptions options;
  options.timestamp_us = timestamp_us;
  options.metadata = userTimestampMetadata(timestamp_us);
  return options;
}

}  // namespace

std::optional<std::uint64_t> PassthroughBitrateMonitor::record(std::size_t frame_bytes, Clock::time_point now)
{
  if (window_started_at_ == Clock::time_point{}) {
    window_started_at_ = now;
  }
  const auto elapsed = now - window_started_at_;
  if (elapsed < kBitrateWindow) {
    window_bytes_ += frame_bytes;
    return std::nullopt;
  }

  const auto elapsed_us =
    static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
  const std::uint64_t bitrate_bps = window_bytes_ * 8U * 1'000'000U / elapsed_us;
  window_started_at_ = now;
  window_bytes_ = frame_bytes;

  if (
    target_bitrate_bps_ == 0U ||
    bitrate_bps * kOverBudgetRatioDenominator <= target_bitrate_bps_ * kOverBudgetRatioNumerator)
  {
    return std::nullopt;
  }
  return bitrate_bps;
}

std::shared_ptr<TrackPublisher> TrackPublisher::create(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
  RoomConnection & connection,
  StreamSpec spec,
  const SubscriptionQosConfig * qos_config)
{
  auto publisher = std::make_shared<TrackPublisher>(connection, std::move(spec));
  if (std::holds_alternative<OtherInput>(publisher->spec_.input)) {
    auto stream = std::make_unique<GStreamerStream>(publisher->spec_, *publisher);
    stream->start();
    publisher->gstreamer_stream_ = std::move(stream);
    return publisher;
  }

  auto stream = std::make_shared<RosStream>(std::move(node_interfaces), publisher->spec_, qos_config, *publisher);
  stream->start();
  publisher->ros_stream_ = std::move(stream);
  return publisher;
}

TrackPublisher::TrackPublisher(
  RoomConnection & connection,
  StreamSpec spec,
  EncodedFeedbackReader read_encoded_feedback,
  std::chrono::milliseconds key_frame_stall_timeout)
: connection_(connection)
, spec_(std::move(spec))
, read_encoded_feedback_(std::move(read_encoded_feedback))
, key_frame_stall_timeout_(key_frame_stall_timeout)
{
  if (read_encoded_feedback_ == nullptr) {
    read_encoded_feedback_ = [](const livekit::EncodedVideoSource & source) { return source.takeFeedback(); };
  }
}

TrackPublisher::~TrackPublisher()
{
  close();
}

PipelineCallbacks TrackPublisher::makePipelineCallbacks(
  std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed)
{
  PipelineCallbacks callbacks;
  callbacks.is_shutdown = std::move(is_shutdown);
  callbacks.on_frame = [this](const livekit::VideoFrame & frame, std::int64_t timestamp_us) {
    capture(frame, timestamp_us);
  };
  callbacks.on_unpack_failed = [this](const std::string & error) { onSampleUnpackFailed(error); };
  callbacks.on_capture_failed = [this](const std::string & error) { onCaptureFailed(error); };
  callbacks.on_failed = std::move(on_failed);
  return callbacks;
}

PipelineCallbacks TrackPublisher::makeEncodedPipelineCallbacks(
  std::function<bool()> is_shutdown,
  std::function<void(const std::string & reason)> on_failed,
  std::function<void(const std::string & reason, const std::string & offered_caps)> on_caps_mismatch,
  std::function<void()> request_key_frame)
{
  PipelineCallbacks callbacks;
  callbacks.is_shutdown = std::move(is_shutdown);
  callbacks.on_unpack_failed = [this](const std::string & error) { onSampleUnpackFailed(error); };
  callbacks.on_capture_failed = [this](const std::string & error) { onCaptureFailed(error); };
  callbacks.on_failed = std::move(on_failed);
  callbacks.on_encoded_frame = [this, request_key_frame](livekit::EncodedVideoSource::Frame frame, bool discontinuous) {
    (void)captureEncoded(std::move(frame), request_key_frame, discontinuous);
  };
  callbacks.on_caps_mismatch = std::move(on_caps_mismatch);
  callbacks.on_key_frame_without_parameter_sets = [this]() { onKeyFrameWithoutParameterSets(); };
  return callbacks;
}

void TrackPublisher::capture(const livekit::VideoFrame & frame, std::int64_t timestamp_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  const int width = frame.width();
  const int height = frame.height();
  if (source_ == nullptr || source_->width() != width || source_->height() != height) {
    const bool republish = published_once_;

    tryUnpublish(connection_, track_);
    track_.reset();
    source_.reset();
    auto source = std::make_shared<livekit::VideoSource>(width, height);
    auto track =
      connection_.publishVideoTrack(spec_.track_name, source, publishOptionsWithFrameMetadata(spec_.publish_options));
    source_ = std::move(source);
    track_ = std::move(track);
    captured_frame_logged_ = false;

    published_once_ = true;
    if (republish) {
      LogEvent(kLogger, "video_stream_track_republished")
        .fieldOr("stream_key", spec_.stream_key)
        .field("width", width)
        .field("height", height)
        .info();
    }
  }

  source_->captureFrame(frame, captureOptions(timestamp_us));
  if (!captured_frame_logged_) {
    LogEvent(kLogger, "video_stream_frame_captured")
      .fieldOr("stream_key", spec_.stream_key)
      .fieldOr("track_name", spec_.track_name)
      .field("width", width)
      .field("height", height)
      .field("timestamp_us", timestamp_us)
      .info();
    captured_frame_logged_ = true;
  }
}

bool TrackPublisher::captureEncoded(
  livekit::EncodedVideoSource::Frame frame, const std::function<void()> & request_key_frame, bool discontinuous)
{
  bool sent = false;
  bool key_frame_needed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return false;
    }
    sent = captureEncodedLocked(std::move(frame), discontinuous);
    // A viewer's request shares the throttle with discarded frames.
    key_frame_needed = (!sent || livekit_key_frame_request_pending_) && keyFrameRequestDueLocked();
    if (key_frame_needed && livekit_key_frame_request_pending_) {
      logViewerKeyFrameRequestLocked();
    }
  }
  // Never call into GStreamer while holding mutex_: the request travels upstream
  // through user-supplied source elements, which may block on their own locks.
  if (key_frame_needed) {
    request_key_frame();
  }
  return sent;
}

bool TrackPublisher::captureEncodedLocked(livekit::EncodedVideoSource::Frame frame, bool discontinuous)
{
  // After a gap delta frames can't be decoded; no change while already awaiting a keyframe.
  const bool follows_publish = std::exchange(just_published_, false);
  if (discontinuous && !awaiting_key_frame_) {
    if (!follows_publish) {
      logDiscontinuityLocked();
    }
    if (!frame.is_keyframe) {
      awaitKeyFrameLocked();
    }
  }
  if (awaiting_key_frame_ && !frame.is_keyframe) {
    ++discarded_frame_count_;
    logWaitingForFirstKeyFrameLocked();
    throwIfKeyFrameStalledLocked();
    return false;
  }

  if (encoded_source_ == nullptr) {
    publishEncodedSourceLocked(frame.width, frame.height);
  }
  if (!sendEncodedFrameLocked(frame)) {
    return false;
  }

  if (!captured_frame_logged_) {
    LogEvent(kLogger, "video_stream_frame_captured")
      .fieldOr("stream_key", spec_.stream_key)
      .fieldOr("track_name", spec_.track_name)
      .field("width", frame.width)
      .field("height", frame.height)
      .field("timestamp_us", frame.timestamp_us)
      .info();
    captured_frame_logged_ = true;
  }
  if (awaiting_key_frame_) {
    awaiting_key_frame_ = false;
    awaiting_since_.reset();
    logResumedLocked();
  }
  // Cleared before reading feedback so a request reported with this keyframe stays pending.
  if (frame.is_keyframe) {
    livekit_key_frame_request_pending_ = false;
  }
  checkEncodedBitrateLocked(readEncodedFeedbackLocked(), frame.data.size());
  return true;
}

livekit::EncodedVideoSource::Feedback TrackPublisher::readEncodedFeedbackLocked()
{
  livekit::EncodedVideoSource::Feedback feedback;
  try {
    feedback = read_encoded_feedback_(*encoded_source_);
  } catch (const std::exception & exc) {
    logThrottledFailureLocked(feedback_failed_log_throttle_, "video_stream_passthrough_feedback_failed", exc.what());
  }
  if (feedback.keyframe_requested) {
    livekit_key_frame_request_pending_ = true;
  }
  return feedback;
}

void TrackPublisher::checkEncodedBitrateLocked(
  const livekit::EncodedVideoSource::Feedback & feedback, std::size_t frame_bytes)
{
  // Without max_bitrate_bps, LiveKit's target is only a default, so exceeding it proves nothing.
  const auto & encoding = spec_.publish_options.video_encoding;
  if (!encoding.has_value()) {
    return;
  }

  if (feedback.rate_control.has_value()) {
    bitrate_monitor_.setTarget(feedback.rate_control->target_bitrate_bps);
  }
  const auto bitrate_bps = bitrate_monitor_.record(frame_bytes, PassthroughBitrateMonitor::Clock::now());
  if (!bitrate_bps.has_value()) {
    return;
  }
  const std::size_t pending = over_budget_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_over_budget")
    .fieldOr("stream_key", spec_.stream_key)
    .field("bitrate_bps", *bitrate_bps)
    .field("target_bitrate_bps", bitrate_monitor_.targetBitrateBps())
    .field("count", pending)
    .warn();
}

void TrackPublisher::logViewerKeyFrameRequestLocked()
{
  const std::size_t pending = viewer_key_frame_request_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_viewer_key_frame_request")
    .fieldOr("stream_key", spec_.stream_key)
    .field("count", pending)
    .info();
}

void TrackPublisher::logThrottledFailureLocked(EventThrottle & throttle, const char * event, const std::string & error)
{
  const std::size_t pending = throttle.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, event)
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .field("count", pending)
    .warn();
}

void TrackPublisher::logWaitingForFirstKeyFrameLocked()
{
  // Only before the first publish, when nothing else says why there is no track.
  if (encoded_source_ != nullptr) {
    return;
  }
  const std::size_t pending = waiting_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_waiting_for_key_frame")
    .fieldOr("stream_key", spec_.stream_key)
    .field("discarded_frames", discarded_frame_count_)
    .field("count", pending)
    .info();
}

void TrackPublisher::logResumedLocked()
{
  // While throttled, keep counting so the next log includes these frames.
  if (discarded_frame_count_ == 0U) {
    return;
  }
  const std::size_t pending = resumed_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_resumed")
    .fieldOr("stream_key", spec_.stream_key)
    .field("discarded_frames", discarded_frame_count_)
    .field("count", pending)
    .info();
  discarded_frame_count_ = 0U;
}

bool TrackPublisher::sendEncodedFrameLocked(livekit::EncodedVideoSource::Frame & frame)
{
  frame.metadata = userTimestampMetadata(frame.timestamp_us);
  std::string error = "LiveKit did not accept the encoded frame.";
  try {
    if (encoded_source_->captureFrame(frame)) {
      return true;
    }
  } catch (const std::invalid_argument & exc) {
    // The SDK rejected this frame. Any other exception is a fault and restarts the pipeline.
    error = exc.what();
  }

  awaiting_key_frame_ = true;
  logThrottledFailureLocked(send_failed_log_throttle_, "video_stream_passthrough_send_failed", error);
  throwIfKeyFrameStalledLocked();
  return false;
}

void TrackPublisher::awaitKeyFrame()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  awaitKeyFrameLocked();
  // Called when the pipeline (re)starts: the stall clock begins with its first frame.
  awaiting_since_.reset();
}

void TrackPublisher::logDiscontinuityLocked()
{
  const std::size_t pending = discontinuity_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_discontinuity")
    .fieldOr("stream_key", spec_.stream_key)
    .field("count", pending)
    .warn();
}

// A failed send sets awaiting_key_frame_ directly: resetting the throttle here would request a
// keyframe right after every failure.
void TrackPublisher::awaitKeyFrameLocked()
{
  if (awaiting_key_frame_) {
    return;
  }

  awaiting_key_frame_ = true;
  // The first discarded frame requests a keyframe immediately; later requests stay throttled.
  key_frame_request_throttle_ = EventThrottle(kKeyFrameRequestInterval);
}

bool TrackPublisher::keyFrameRequestDueLocked()
{
  return key_frame_request_throttle_.record() > 0U;
}

// Throwing restarts the pipeline, the only way out when a source never sends a keyframe again.
void TrackPublisher::throwIfKeyFrameStalledLocked()
{
  // Before the first publish the source may just not be ready.
  if (encoded_source_ == nullptr) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!awaiting_since_.has_value()) {
    awaiting_since_ = now;
  }
  const auto waited = now - *awaiting_since_;
  if (waited < key_frame_stall_timeout_) {
    return;
  }
  throw std::runtime_error(
    "no keyframe for " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(waited).count()) + " ms");
}

void TrackPublisher::publishEncodedSourceLocked(std::uint32_t width, std::uint32_t height)
{
  auto source = std::make_shared<livekit::EncodedVideoSource>(
    livekit::VideoCodec::H264, static_cast<int>(width), static_cast<int>(height));
  auto track =
    connection_.publishVideoTrack(spec_.track_name, source, publishOptionsWithFrameMetadata(spec_.publish_options));
  encoded_source_ = std::move(source);
  track_ = std::move(track);
  just_published_ = true;
}

void TrackPublisher::close()
{
  // Stop streams outside mutex_; their callbacks can re-enter this publisher.
  std::shared_ptr<RosStream> ros_stream;
  std::unique_ptr<GStreamerStream> gstreamer_stream;
  std::shared_ptr<livekit::VideoSource> source;
  std::shared_ptr<livekit::EncodedVideoSource> encoded_source;
  std::shared_ptr<livekit::LocalVideoTrack> track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    published_once_ = false;
    captured_frame_logged_ = false;
    closed_ = true;
    ros_stream = std::move(ros_stream_);
    gstreamer_stream = std::move(gstreamer_stream_);
    source = std::move(source_);
    encoded_source = std::move(encoded_source_);
    track = std::move(track_);
  }

  if (ros_stream != nullptr) {
    ros_stream->close();
  }
  if (gstreamer_stream != nullptr) {
    gstreamer_stream->close();
  }
  tryUnpublish(connection_, track);
  track.reset();
  source.reset();
  encoded_source.reset();
}

void TrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_sample_unpack_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::onCaptureFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_capture_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::onKeyFrameWithoutParameterSets()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_ || parameter_sets_warning_logged_) {
    return;
  }

  parameter_sets_warning_logged_ = true;
  LogEvent(kLogger, "video_stream_passthrough_missing_parameter_sets")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldQuoted(
      "hint",
      "a keyframe arrived without SPS and PPS, so viewers who join late or reconnect may not be able to decode "
      "the video; end the source with h264parse config-interval=-1")
    .warn();
}

void TrackPublisher::onRestartFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  const std::size_t pending = restart_failed_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_restart_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .field("count", pending)
    .warn();
}

void TrackPublisher::onAppsrcPushFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "video_stream_push_failed").fieldOr("stream_key", spec_.stream_key).fieldOr("error", error).warn();
}

}  // namespace livekit_ros2_bridge::video
