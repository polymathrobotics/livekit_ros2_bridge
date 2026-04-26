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

#include "video/gstreamer_stream.hpp"

#include <chrono>
#include <stdexcept>
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
, pipeline_(publisher.makePipelineCallbacks(
    [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      return is_shutdown_;
    },
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
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  startPipelineLocked();
}

void GStreamerStream::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
  }

  failure_handler_.close();
  pipeline_.stop();
}

void GStreamerStream::onPipelineFailure(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || !pipeline_.isActive()) {
    return;
  }
  if (!failure_handler_.schedule()) {
    return;
  }

  LogEvent(kLogger, "video_stream_pipeline_recovery_scheduled")
    .field("stream_key", spec_.stream_key)
    .field("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .warn();
}

void GStreamerStream::restartPipelineAfterFailure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
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
  const auto & input = requireOtherInput(spec_);
  pipeline_.start(buildPipelineDescription(input.source_fragment, input.transform_fragment), false);
}

}  // namespace livekit_ros2_bridge::video
