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

#include "video/gstreamer_stream.hpp"

#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video/pipeline_description.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.gstreamer_video_stream");
constexpr auto kRestartDelay = std::chrono::milliseconds(250);

}  // namespace

GStreamerStream::GStreamerStream(StreamSpec spec, TrackPublisher & publisher)
: spec_(std::move(spec))
, publisher_(publisher)
, pipeline_(makePipelineCallbacks())
, failure_handler_(kRestartDelay, [this]() { restartPipelineAfterFailure(); })
{}

GStreamerStream::~GStreamerStream()
{
  close();
}

void GStreamerStream::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    throw std::runtime_error("Video stream is shut down.");
  }

  startPipelineLocked();
}

void GStreamerStream::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_.load(std::memory_order_acquire)) {
      return;
    }

    is_shutdown_.store(true, std::memory_order_release);
  }

  failure_handler_.close();
  pipeline_.stop();
}

PipelineCallbacks GStreamerStream::makePipelineCallbacks()
{
  auto is_shutdown = [this]() { return is_shutdown_.load(std::memory_order_acquire); };
  auto on_failed = [this](const std::string & reason) { onPipelineFailure(reason); };
  if (!requireOtherInput(spec_).passthrough) {
    return publisher_.makePipelineCallbacks(is_shutdown, on_failed);
  }

  // Requesting a keyframe must not take mutex_: it runs inside pipeline
  // callbacks, which pipeline_.stop() waits for.
  return publisher_.makeEncodedPipelineCallbacks(
    is_shutdown,
    on_failed,
    [this](const std::string & reason, const std::string & offered_caps) {
      onPipelineFailure(reason, true, offered_caps);
    },
    [this]() { pipeline_.requestKeyFrame(); });
}

// This path must stay lock-free: the sync bus handler can deliver a failure
// from inside start() or restartPipelineAfterFailure(), which hold mutex_ while
// GStreamer performs the state change. Locking here would deadlock against the
// calling thread itself. schedule() coalesces duplicate failures, and close()
// marks the handler closed before teardown, so no stream mutex_ is needed.
void GStreamerStream::onPipelineFailure(
  const std::string & reason, bool caps_mismatch, const std::string & offered_caps)
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (!failure_handler_.schedule()) {
    return;
  }
  std::size_t pending = 0U;
  {
    std::lock_guard<std::mutex> lock(recovery_log_mutex_);
    pending = recovery_log_throttle_.record();
  }
  if (pending == 0U) {
    return;
  }

  LogEvent event(kLogger, "video_stream_pipeline_recovery_scheduled");
  event.field("stream_key", spec_.stream_key)
    .field("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .field("count", pending);
  if (caps_mismatch) {
    event.fieldQuoted("hint", "the source may not be H.264; passthrough supports H.264 only");
  }
  if (!offered_caps.empty()) {
    event.fieldQuoted("offered_caps", offered_caps);
  }
  event.warn();
}

void GStreamerStream::restartPipelineAfterFailure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  pipeline_.stop();
  // A passthrough pipeline may start mid-stream; viewers need a keyframe first.
  if (requireOtherInput(spec_).passthrough) {
    publisher_.awaitKeyFrame();
  }
  try {
    startPipelineLocked();
  } catch (const std::exception & exc) {
    publisher_.onRestartFailed(exc.what());
  }
}

void GStreamerStream::startPipelineLocked()
{
  const auto & input = requireOtherInput(spec_);
  if (input.passthrough) {
    pipeline_.start(buildPassthroughPipelineDescription(input.source_fragment), false, PipelineOutput::EncodedH264);
    return;
  }
  pipeline_.start(buildPipelineDescription(input.source_fragment, input.transform_fragment), false);
}

}  // namespace livekit_ros2_bridge::video
