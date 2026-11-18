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
  RoomConnection & connection, StreamSpec spec, EncodedFeedbackReader read_encoded_feedback)
: connection_(connection)
, spec_(std::move(spec))
, read_encoded_feedback_(std::move(read_encoded_feedback))
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
    return false;
  }

  // The failure restarts the pipeline; frames arriving before then must not follow a lost one.
  try {
    sendEncodedFrameLocked(frame);
  } catch (...) {
    awaitKeyFrameLocked();
    throw;
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
    logResumedLocked();
  }
  // Cleared before reading feedback so a request reported with this keyframe stays pending.
  if (frame.is_keyframe) {
    livekit_key_frame_request_pending_ = false;
  }
  readEncodedFeedbackLocked();
  return true;
}

void TrackPublisher::readEncodedFeedbackLocked()
{
  try {
    if (read_encoded_feedback_(*encoded_source_).keyframe_requested) {
      livekit_key_frame_request_pending_ = true;
    }
  } catch (const std::exception & exc) {
    logFeedbackFailedLocked(exc.what());
  }
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

void TrackPublisher::logFeedbackFailedLocked(const std::string & error)
{
  const std::size_t pending = feedback_failed_log_throttle_.record();
  if (pending == 0U) {
    return;
  }

  LogEvent(kLogger, "video_stream_passthrough_feedback_failed")
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
