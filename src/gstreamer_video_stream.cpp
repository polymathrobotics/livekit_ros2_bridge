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

#include "gstreamer_video_stream.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video_pipeline_description.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.gstreamer_video_stream");
constexpr auto kRestartDelay = std::chrono::milliseconds(250);

}  // namespace

GStreamerVideoStream::GStreamerVideoStream(VideoStreamSpec spec, VideoTrackPublisher & publisher)
: spec_(std::move(spec))
, publisher_(publisher)
, pipeline_(
    spec_,
    GStreamerPipelineCallbacks{
      [this]() { return isShutdown(); },
      [this](const livekit::VideoFrame & frame, std::int64_t timestamp_us) {
        publisher_.captureFrame(frame, timestamp_us);
      },
      [this](const std::string & error) { publisher_.onSampleUnpackFailed(error); },
      [this](const std::string & error) { publisher_.onCaptureFailed(error); },
      [this](const std::string & reason) { onPipelineFailure(reason); },
    })
{}

GStreamerVideoStream::~GStreamerVideoStream()
{
  close();
}

void GStreamerVideoStream::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    throw std::runtime_error("Video stream is shut down.");
  }

  pipeline_.start(buildDescription(), false);
}

void GStreamerVideoStream::close()
{
  std::thread restart_thread;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    restart_condition_.notify_all();
    restart_thread = std::move(restart_thread_);
  }

  // The restart worker captures this, so join it before members destruct.
  if (restart_thread.joinable()) {
    restart_thread.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pipeline_.isActive() || restart_pending_) {
      LogEvent(kLogger, "video_stream_source_shutdown")
        .field("stream_key", spec_.stream_key)
        .fieldIf(restart_pending_, "restart_pending", true)
        .info();
    }
  }

  pipeline_.stop();
}

void GStreamerVideoStream::onPipelineFailure(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || !pipeline_.isActive()) {
    return;
  }
  if (restart_pending_) {
    return;
  }

  restart_pending_ = true;
  LogEvent(kLogger, "video_stream_pipeline_recovery_scheduled")
    .field("stream_key", spec_.stream_key)
    .field("track_name", spec_.track_name)
    .field("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .warn();

  if (!restart_thread_.joinable()) {
    restart_thread_ = std::thread([this]() { restartLoop(); });
  }
  restart_condition_.notify_one();
}

std::string GStreamerVideoStream::buildDescription() const
{
  const auto & input = requireOtherVideoInput(spec_);
  return buildPipelineDescription(input.ingress_fragment, input.transform_fragment);
}

bool GStreamerVideoStream::isShutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

void GStreamerVideoStream::restartLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    restart_condition_.wait(lock, [this]() { return is_shutdown_ || restart_pending_; });
    if (is_shutdown_) {
      return;
    }

    if (restart_condition_.wait_for(lock, kRestartDelay, [this]() { return is_shutdown_; })) {
      return;
    }

    lock.unlock();
    restart();
    lock.lock();
  }
}

void GStreamerVideoStream::restart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }

  pipeline_.stop();
  restart_pending_ = false;

  try {
    pipeline_.start(buildDescription(), false);
  } catch (const std::exception & exc) {
    publisher_.onRestartFailed(exc.what());
  }
}

}  // namespace livekit_ros2_bridge
