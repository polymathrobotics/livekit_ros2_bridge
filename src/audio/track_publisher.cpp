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

#include "audio/track_publisher.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "audio/gstreamer_pipeline.hpp"
#include "audio/gstreamer_stream.hpp"
#include "livekit/local_audio_track.h"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

const auto kLogger = rclcpp::get_logger("audio_track_publisher");
// Buffered AudioSource mode. A GStreamer source can deliver variable-size buffers
// (not fixed 10 ms frames), and buffered mode smooths those for WebRTC. The LiveKit
// header recommends queue_size_ms == 0 for hardware-paced capture; this is a
// deliberate exception for a GStreamer producer. A capture tie-up is recovered by
// dropping frames (see capture()), never by tearing down the pipeline.
constexpr int kAudioSourceQueueSizeMs = 100;
constexpr auto kFailureLogInterval = std::chrono::seconds(1);

void tryUnpublish(RoomConnection & connection, const std::shared_ptr<livekit::LocalAudioTrack> & track) noexcept
{
  if (track == nullptr) {
    return;
  }

  try {
    connection.unpublishAudioTrack(track);
  } catch (...) {
  }
}

}  // namespace

std::shared_ptr<TrackPublisher> TrackPublisher::create(
  RoomConnection & connection, StreamSpec spec, std::chrono::milliseconds publish_retry_interval)
{
  auto publisher = std::make_shared<TrackPublisher>(connection, std::move(spec), publish_retry_interval);
  auto stream = std::make_unique<GStreamerStream>(publisher->spec_, *publisher);
  stream->start();
  publisher->gstreamer_stream_ = std::move(stream);
  return publisher;
}

TrackPublisher::TrackPublisher(
  RoomConnection & connection, StreamSpec spec, std::chrono::milliseconds publish_retry_interval)
: connection_(connection)
, spec_(std::move(spec))
, publish_retry_interval_(publish_retry_interval)
{}

TrackPublisher::~TrackPublisher()
{
  close();
}

PipelineCallbacks TrackPublisher::makePipelineCallbacks(
  std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed)
{
  return PipelineCallbacks{
    std::move(is_shutdown),
    [this](const livekit::AudioFrame & frame) { capture(frame); },
    [this](const std::string & error) { onSampleUnpackFailed(error); },
    [this](const std::string & error) { onCaptureFailed(error); },
    std::move(on_failed),
  };
}

void TrackPublisher::capture(const livekit::AudioFrame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  if (source_ == nullptr) {
    // The bridge tail forces mono 48 kHz, so the first sample's caps define the
    // AudioSource for the life of the stream. A failed publish is a LiveKit-side
    // hiccup, not a pipeline fault: keep the pipeline alive, drop this frame, and
    // retry the publish after a short backoff. Escalating to a GStreamer error
    // would tear the pipeline down and immediately fail again on its first frame,
    // producing an endless stop/start cycle for a source that is perfectly fine.
    //
    // No bridge-side republish machinery exists on purpose: livekit-client-sdk-cpp
    // v1.6.0 (Rust core handle_restarted, submodule commit 06371a3) auto-republishes
    // every local track after a full reconnect, reusing bound sources. Residual
    // risk is an SDK republish failure, which surfaces as a silent track gap
    // rather than being handled here.
    const auto now = std::chrono::steady_clock::now();
    if (now < next_publish_attempt_) {
      return;
    }

    try {
      auto source =
        std::make_shared<livekit::AudioSource>(frame.sampleRate(), frame.numChannels(), kAudioSourceQueueSizeMs);
      auto track = connection_.publishAudioTrack(spec_.track_name, source, spec_.publish_options);
      source_ = std::move(source);
      track_ = std::move(track);
      captured_frame_logged_ = false;
    } catch (const std::exception & exc) {
      next_publish_attempt_ = now + publish_retry_interval_;
      logTransientFailure("audio_stream_publish_failed_retry", exc.what());
      return;
    }
  }

  try {
    source_->captureFrame(frame);
  } catch (const std::exception & exc) {
    // captureFrame can time out when the LiveKit sink is backpressured; drop this
    // frame but keep the pipeline and track alive rather than failing the stream.
    logTransientFailure("audio_stream_capture_failed_drop", exc.what());
    return;
  }

  if (!captured_frame_logged_) {
    LogEvent(kLogger, "audio_stream_frame_captured")
      .fieldOr("stream_key", spec_.stream_key)
      .fieldOr("track_name", spec_.track_name)
      .field("sample_rate", frame.sampleRate())
      .field("channels", frame.numChannels())
      .info();
    captured_frame_logged_ = true;
  }
}

void TrackPublisher::logTransientFailure(const char * event_name, const std::string & error)
{
  const auto now = std::chrono::steady_clock::now();
  if (now < next_failure_log_) {
    return;
  }
  next_failure_log_ = now + kFailureLogInterval;

  LogEvent(kLogger, event_name)
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("track_name", spec_.track_name)
    .fieldOr("error", error)
    .warn();
}

void TrackPublisher::close()
{
  // Stop streams outside mutex_; their callbacks can re-enter this publisher.
  std::unique_ptr<GStreamerStream> gstreamer_stream;
  std::shared_ptr<livekit::AudioSource> source;
  std::shared_ptr<livekit::LocalAudioTrack> track;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    captured_frame_logged_ = false;
    closed_ = true;
    gstreamer_stream = std::move(gstreamer_stream_);
    source = std::move(source_);
    track = std::move(track_);
  }

  if (gstreamer_stream != nullptr) {
    gstreamer_stream->close();
  }
  tryUnpublish(connection_, track);
  track.reset();
  source.reset();
}

void TrackPublisher::onSampleUnpackFailed(const std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return;
  }

  LogEvent(kLogger, "audio_stream_sample_unpack_failed")
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

  LogEvent(kLogger, "audio_stream_capture_failed")
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

  LogEvent(kLogger, "audio_stream_restart_failed")
    .fieldOr("stream_key", spec_.stream_key)
    .fieldOr("error", error)
    .warn();
}

}  // namespace livekit_ros2_bridge::audio
