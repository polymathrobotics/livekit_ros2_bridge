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
#include "utils/scope_exit.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/gstreamer_stream.hpp"
#include "video/ros_stream.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

const auto kLogger = rclcpp::get_logger("video_track_publisher");
constexpr auto kBitrateWindow = std::chrono::seconds(1);
constexpr auto kOverBudgetDuration = std::chrono::seconds(3);

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
  const Clock::time_point window_started_at = window_started_at_;
  window_started_at_ = now;
  window_bytes_ = frame_bytes;

  // Well above means more than 1.5x the target.
  if (target_bitrate_bps_ == 0U || bitrate_bps * 2U <= target_bitrate_bps_ * 3U) {
    over_budget_since_.reset();
    return std::nullopt;
  }
  if (!over_budget_since_.has_value()) {
    over_budget_since_ = window_started_at;
  }
  if (now - *over_budget_since_ < kOverBudgetDuration) {
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

TrackPublisher::TrackPublisher(RoomConnection & connection, StreamSpec spec)
: connection_(connection)
, spec_(std::move(spec))
{}

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
  callbacks.on_encoded_frame = [this, request_key_frame](livekit::EncodedVideoSource::Frame frame) {
    captureEncoded(std::move(frame), request_key_frame);
  };
  callbacks.on_fell_behind = [this, request_key_frame]() { onFellBehind(request_key_frame); };
  callbacks.on_caps_mismatch = std::move(on_caps_mismatch);
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

void TrackPublisher::captureEncoded(
  livekit::EncodedVideoSource::Frame frame, const std::function<void()> & request_key_frame)
{
  bool key_frame_needed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    key_frame_needed = captureEncodedLocked(std::move(frame));
  }
  // Never call into GStreamer while holding mutex_: the request travels upstream
  // through user-supplied source elements, and onFellBehind() waits for mutex_ on
  // an upstream streaming thread.
  if (key_frame_needed) {
    request_key_frame();
  }
}

bool TrackPublisher::captureEncodedLocked(livekit::EncodedVideoSource::Frame frame)
{
  if (closed_) {
    return false;
  }

  if (awaiting_key_frame_ && !frame.is_keyframe) {
    ++discarded_frame_count_;
    return keyFrameRequestDueLocked();
  }

  // A failed publish or send must not fail the GStreamer pipeline: viewers need a
  // keyframe after it either way, so wait for the next one and try again then.
  try {
    sendEncodedFrameLocked(frame);
  } catch (const std::exception & exc) {
    awaitKeyFrameLocked();
    logCaptureFailedLocked(exc.what());
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
    // While the log is throttled, keep counting so the next one includes these frames.
    const std::size_t pending = discarded_frame_count_ > 0U ? resumed_log_throttle_.record() : 0U;
    if (pending > 0U) {
      LogEvent(kLogger, "video_stream_passthrough_resumed")
        .fieldOr("stream_key", spec_.stream_key)
        .field("discarded_frames", discarded_frame_count_)
        .field("count", pending)
        .info();
      discarded_frame_count_ = 0U;
    }
  }

  return handleEncodedFeedbackLocked(frame.data.size());
}

void TrackPublisher::sendEncodedFrameLocked(livekit::EncodedVideoSource::Frame & frame)
{
  if (encoded_source_ == nullptr) {
    publishEncodedSourceLocked(frame.width, frame.height);
  }

  frame.metadata = userTimestampMetadata(frame.timestamp_us);
  if (!encoded_source_->captureFrame(frame)) {
    throw std::runtime_error("LiveKit did not accept the encoded frame.");
  }
}

void TrackPublisher::awaitKeyFrame()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  awaitKeyFrameLocked();
}

void TrackPublisher::onFellBehind(const std::function<void()> & request_key_frame)
{
  // The first publish blocks the sample thread on the server while the source
  // keeps producing, so overruns then don't mean the consumer fell behind.
  // Checked before mutex_, which the publish holds.
  if (publishing_encoded_source_.load(std::memory_order_acquire)) {
    return;
  }

  bool key_frame_needed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    // Frames are discarded quickly while waiting, so the queue drains and the
    // feed resumes live at the requested keyframe.
    awaitKeyFrameLocked();
    key_frame_needed = keyFrameRequestDueLocked();
    if (const std::size_t pending = fell_behind_log_throttle_.record(); pending > 0U) {
      LogEvent(kLogger, "video_stream_passthrough_fell_behind")
        .fieldOr("stream_key", spec_.stream_key)
        .field("count", pending)
        .warn();
    }
  }
  // Called outside mutex_; see captureEncoded().
  if (key_frame_needed) {
    request_key_frame();
  }
}

void TrackPublisher::awaitKeyFrameLocked()
{
  if (awaiting_key_frame_) {
    return;
  }

  awaiting_key_frame_ = true;
  // Let the first discarded frame ask for a keyframe right away; requests while
  // still waiting stay throttled.
  key_frame_request_throttle_ = EventThrottle(kKeyFrameRequestInterval);
}

bool TrackPublisher::keyFrameRequestDueLocked()
{
  return key_frame_request_throttle_.record() > 0U;
}

void TrackPublisher::publishEncodedSourceLocked(std::uint32_t width, std::uint32_t height)
{
  auto source = std::make_shared<livekit::EncodedVideoSource>(
    livekit::VideoCodec::H264, static_cast<int>(width), static_cast<int>(height));
  publishing_encoded_source_.store(true, std::memory_order_release);
  const ScopeExit publishing_exit([this]() { publishing_encoded_source_.store(false, std::memory_order_release); });
  auto track =
    connection_.publishVideoTrack(spec_.track_name, source, publishOptionsWithFrameMetadata(spec_.publish_options));
  encoded_source_ = std::move(source);
  track_ = std::move(track);
  captured_frame_logged_ = false;
  published_once_ = true;
}

bool TrackPublisher::handleEncodedFeedbackLocked(std::size_t frame_bytes)
{
  // Feedback only updates as frames are pushed, so poll it after each one. It is
  // advisory: failing to read it doesn't fail the frame that was just sent.
  livekit::EncodedVideoSource::Feedback feedback;
  try {
    feedback = encoded_source_->takeFeedback();
  } catch (const std::exception & exc) {
    if (const std::size_t pending = feedback_failed_log_throttle_.record(); pending > 0U) {
      LogEvent(kLogger, "video_stream_passthrough_feedback_failed")
        .fieldOr("stream_key", spec_.stream_key)
        .fieldOr("error", exc.what())
        .field("count", pending)
        .warn();
    }
  }
  if (feedback.rate_control.has_value()) {
    bitrate_monitor_.setTarget(feedback.rate_control->target_bitrate_bps);
  }

  const auto bitrate_bps = bitrate_monitor_.record(frame_bytes, PassthroughBitrateMonitor::Clock::now());
  const std::size_t pending = bitrate_bps.has_value() ? over_budget_log_throttle_.record() : 0U;
  if (pending > 0U) {
    LogEvent(kLogger, "video_stream_passthrough_over_budget")
      .fieldOr("stream_key", spec_.stream_key)
      .field("bitrate_bps", *bitrate_bps)
      .field("target_bitrate_bps", bitrate_monitor_.targetBitrateBps())
      .field("count", pending)
      .warn();
  }
  // Shares the throttle with discarded frames so the source is asked at most once per interval.
  return feedback.keyframe_requested && keyFrameRequestDueLocked();
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

  logCaptureFailedLocked(error);
}

void TrackPublisher::logCaptureFailedLocked(const std::string & error)
{
  LogEvent(kLogger, "video_stream_capture_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
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
