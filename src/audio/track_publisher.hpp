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

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "audio/stream_spec.hpp"
#include "core/room_connection.hpp"
#include "livekit/audio_frame.h"
#include "livekit/audio_source.h"

namespace livekit_ros2_bridge::audio
{

class GStreamerStream;
struct PipelineCallbacks;

class TrackPublisher final
{
public:
  // Backoff before a capture retries the publish after a LiveKit publish failure.
  static constexpr std::chrono::milliseconds kDefaultPublishRetryInterval{250};

  static std::shared_ptr<TrackPublisher> create(
    RoomConnection & connection,
    StreamSpec spec,
    std::chrono::milliseconds publish_retry_interval = kDefaultPublishRetryInterval);

  // Does not start a GStreamer input stream.
  TrackPublisher(
    RoomConnection & connection,
    StreamSpec spec,
    std::chrono::milliseconds publish_retry_interval = kDefaultPublishRetryInterval);

  ~TrackPublisher();

  TrackPublisher(const TrackPublisher &) = delete;
  TrackPublisher & operator=(const TrackPublisher &) = delete;
  TrackPublisher(TrackPublisher &&) = delete;
  TrackPublisher & operator=(TrackPublisher &&) = delete;

  const StreamSpec & spec() const
  {
    return spec_;
  }

  // First frame publishes; the AudioSource is created lazily from the first
  // sample's caps. The bridge tail forces mono 48 kHz, so no republish path
  // is needed on rate or channel change.
  void capture(const livekit::AudioFrame & frame);

private:
  friend class GStreamerStream;

  // May be invoked from GStreamer or LiveKit worker threads after close() starts.
  void onSampleUnpackFailed(const std::string & error);
  void onCaptureFailed(const std::string & error);
  void onRestartFailed(const std::string & error);

  PipelineCallbacks makePipelineCallbacks(
    std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed);

  // Logs a transient LiveKit failure at most once per interval. Caller holds mutex_.
  void logTransientFailure(const char * event_name, const std::string & error);

  void close();

  RoomConnection & connection_;
  StreamSpec spec_;

  // Guards stream handles, publication state, and late callbacks racing with close().
  mutable std::mutex mutex_;
  bool closed_ = false;
  bool captured_frame_logged_ = false;
  // Backoff before the next (re)publish attempt after a LiveKit publish failure. Keeps a
  // dead room from turning a healthy pipeline into a per-frame publish storm.
  std::chrono::milliseconds publish_retry_interval_;
  std::chrono::steady_clock::time_point next_publish_attempt_{};
  // Throttle for transient-failure logging so a persistent hiccup cannot flood the log.
  std::chrono::steady_clock::time_point next_failure_log_{};
  std::unique_ptr<GStreamerStream> gstreamer_stream_;
  std::shared_ptr<livekit::AudioSource> source_;
  std::shared_ptr<livekit::LocalAudioTrack> track_;
};

}  // namespace livekit_ros2_bridge::audio
