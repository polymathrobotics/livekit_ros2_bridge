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

#include "audio/gstreamer_stream.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "audio/pipeline_description.hpp"
#include "audio/track_publisher.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.gstreamer_audio_stream");
constexpr auto kRestartDelay = std::chrono::milliseconds(250);

}  // namespace

GStreamerStream::GStreamerStream(StreamSpec spec, TrackPublisher & publisher)
: spec_(std::move(spec))
, publisher_(publisher)
, pipeline_(publisher.makePipelineCallbacks(
    [this]() { return is_shutdown_.load(std::memory_order_acquire); },
    [this](const std::string & reason) { onPipelineFailure(reason); }))
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
    throw std::runtime_error("Audio stream is shut down.");
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

// This path must stay lock-free: the sync bus handler can deliver a failure
// from inside start() or restartPipelineAfterFailure(), which hold mutex_ while
// GStreamer performs the state change. Locking here would deadlock against the
// calling thread itself. schedule() coalesces duplicate failures, and close()
// marks the handler closed before teardown, so no stream mutex_ is needed.
void GStreamerStream::onPipelineFailure(const std::string & reason)
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (!failure_handler_.schedule()) {
    return;
  }

  LogEvent(kLogger, "audio_stream_pipeline_recovery_scheduled")
    .field("stream_key", spec_.stream_key)
    .field("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .warn();
}

void GStreamerStream::restartPipelineAfterFailure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  pipeline_.stop();
  try {
    startPipelineLocked();
  } catch (const std::exception & exc) {
    publisher_.onRestartFailed(exc.what());
  }
}

void GStreamerStream::startPipelineLocked()
{
  const auto & input = spec_.input;
  pipeline_.start(buildPipelineDescription(input.source_fragment, input.transform_fragment));
}

}  // namespace livekit_ros2_bridge::audio
