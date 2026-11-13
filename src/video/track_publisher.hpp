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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "core/room_connection.hpp"
#include "core/subscription_qos.hpp"
#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "utils/event_throttle.hpp"
#include "video/stream_spec.hpp"

namespace livekit_ros2_bridge::video
{

class GStreamerStream;
struct PipelineCallbacks;
class RosStream;

class TrackPublisher final
{
public:
  static std::shared_ptr<TrackPublisher> create(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeGraphInterface> node_interfaces,
    RoomConnection & connection,
    StreamSpec spec,
    const SubscriptionQosConfig * qos_config);

  // Does not start a ROS or GStreamer input stream.
  TrackPublisher(RoomConnection & connection, StreamSpec spec);

  ~TrackPublisher();

  TrackPublisher(const TrackPublisher &) = delete;
  TrackPublisher & operator=(const TrackPublisher &) = delete;
  TrackPublisher(TrackPublisher &&) = delete;
  TrackPublisher & operator=(TrackPublisher &&) = delete;

  const StreamSpec & spec() const
  {
    return spec_;
  }

  // First frame publishes; size changes recreate the LiveKit VideoSource.
  void capture(const livekit::VideoFrame & frame, std::int64_t timestamp_us);

  // Passthrough: publishes an EncodedVideoSource on the first keyframe and discards frames (asking
  // for a keyframe) whenever it waits for one. A failed publish or send throws, and frames are then
  // discarded until the next keyframe.
  void captureEncoded(livekit::EncodedVideoSource::Frame frame, const std::function<void()> & request_key_frame);
  // Passthrough: discard frames until the next keyframe, e.g. after a pipeline restart.
  void awaitKeyFrame();
  // Passthrough: the pipeline queue filled up; skip ahead to a requested keyframe. A publish stalls
  // the queue itself, so this waits for one in progress and is ignored for kBridgeQueueDuration after.
  void onFellBehind(const std::function<void()> & request_key_frame);

private:
  friend class GStreamerStream;
  friend class RosStream;

  // May be invoked from ROS, GStreamer, or LiveKit worker threads after close() starts.
  void onSampleUnpackFailed(const std::string & error);
  void onCaptureFailed(const std::string & error);
  void onKeyFrameWithoutParameterSets();
  void onRestartFailed(const std::string & error);
  void onAppsrcPushFailed(const std::string & error);

  // Callbacks for RawI420 pipelines.
  PipelineCallbacks makePipelineCallbacks(
    std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed);
  // Callbacks for EncodedH264 (passthrough) pipelines; request_key_frame asks the
  // source for a keyframe.
  PipelineCallbacks makeEncodedPipelineCallbacks(
    std::function<bool()> is_shutdown,
    std::function<void(const std::string & reason)> on_failed,
    std::function<void(const std::string & reason, const std::string & offered_caps)> on_caps_mismatch,
    std::function<void()> request_key_frame);

  void close();

  // The passthrough *Locked helpers return whether to request a keyframe; callers
  // do so after releasing mutex_ (see captureEncoded()).
  bool captureEncodedLocked(livekit::EncodedVideoSource::Frame frame);
  void logWaitingForFirstKeyFrameLocked();
  // Publishes the encoded source on first use; throws if the publish or send fails.
  void sendEncodedFrameLocked(livekit::EncodedVideoSource::Frame & frame);
  void awaitKeyFrameLocked();
  bool keyFrameRequestDueLocked();
  void publishEncodedSourceLocked(std::uint32_t width, std::uint32_t height);
  void logResumedLocked();
  void logFellBehindLocked();

  static constexpr auto kKeyFrameRequestInterval = std::chrono::milliseconds(500);
  static constexpr auto kLogThrottle = std::chrono::seconds(5);

  RoomConnection & connection_;
  StreamSpec spec_;

  // Guards stream handles, publication state, and late callbacks racing with close().
  std::mutex mutex_;
  bool closed_ = false;
  bool published_once_ = false;
  bool captured_frame_logged_ = false;
  std::shared_ptr<RosStream> ros_stream_;
  // A source that stays broken fails every restart; log it once per interval.
  EventThrottle restart_failed_log_throttle_{kLogThrottle};
  std::unique_ptr<GStreamerStream> gstreamer_stream_;
  std::shared_ptr<livekit::VideoSource> source_;
  std::shared_ptr<livekit::LocalVideoTrack> track_;

  // Passthrough state.
  std::shared_ptr<livekit::EncodedVideoSource> encoded_source_;
  // When the last publish ended, successful or not; min() before the first one. Set before the
  // publish releases mutex_, so an overrun that waited for it falls inside the ignored window.
  std::chrono::steady_clock::time_point encoded_publish_ended_at_ = std::chrono::steady_clock::time_point::min();
  bool awaiting_key_frame_ = true;
  // Missing parameter sets come from the source's configuration, so warn once; the publisher
  // outlives pipeline restarts.
  bool parameter_sets_warning_logged_ = false;
  std::size_t discarded_frame_count_ = 0U;
  EventThrottle key_frame_request_throttle_{kKeyFrameRequestInterval};
  EventThrottle fell_behind_log_throttle_{kLogThrottle};
  EventThrottle resumed_log_throttle_{kLogThrottle};
  EventThrottle waiting_log_throttle_{kLogThrottle};
};

}  // namespace livekit_ros2_bridge::video
