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
    GStreamerPipelineCallbacks{
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return is_shutdown_;
      },
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

  const auto & input = requireOtherVideoInput(spec_);
  pipeline_.start(buildPipelineDescription(input.ingress_fragment, input.transform_fragment), false);
}

void GStreamerVideoStream::close()
{
  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    is_shutdown_ = true;
    restart_condition_.notify_all();
    worker = std::move(restart_worker_);
  }

  // The restart worker captures this, so join it before members destruct.
  if (worker.joinable()) {
    worker.join();
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
    .field("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .warn();

  if (!restart_worker_.joinable()) {
    restart_worker_ = std::thread([this]() { runRestartLoop(); });
  }
  restart_condition_.notify_one();
}

void GStreamerVideoStream::runRestartLoop()
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
    lock.lock();
    if (is_shutdown_) {
      return;
    }

    pipeline_.stop();
    restart_pending_ = false;

    try {
      const auto & input = requireOtherVideoInput(spec_);
      pipeline_.start(buildPipelineDescription(input.ingress_fragment, input.transform_fragment), false);
    } catch (const std::exception & exc) {
      publisher_.onRestartFailed(exc.what());
    }
  }
}

}  // namespace livekit_ros2_bridge
