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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
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

// Measures a passthrough source's bitrate in one-second windows against LiveKit's
// latest target. Passthrough can't re-rate the camera, so this only feeds a warning.
class PassthroughBitrateMonitor final
{
public:
  using Clock = std::chrono::steady_clock;

  void setTarget(std::uint64_t target_bitrate_bps)
  {
    target_bitrate_bps_ = target_bitrate_bps;
  }

  std::uint64_t targetBitrateBps() const
  {
    return target_bitrate_bps_;
  }

  // Returns the measured bitrate when it has stayed above 1.5x the target for a few seconds.
  std::optional<std::uint64_t> record(std::size_t frame_bytes, Clock::time_point now);

private:
  std::uint64_t target_bitrate_bps_ = 0U;
  Clock::time_point window_started_at_{};
  std::uint64_t window_bytes_ = 0U;
  std::optional<Clock::time_point> over_budget_since_;
};

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

  // Passthrough: the first keyframe publishes an EncodedVideoSource; size changes
  // don't republish. A viewer can only start decoding at a keyframe, so frames
  // are discarded (asking the source for a keyframe) until one arrives after
  // startup, awaitKeyFrame(), onFellBehind(), or a failed publish or send. Those
  // failures are logged, not thrown, so the pipeline keeps running. A failed
  // publish is retried at the first keyframe after kPublishRetryInterval.
  void captureEncoded(livekit::EncodedVideoSource::Frame frame, const std::function<void()> & request_key_frame);
  // Passthrough: discard frames until the next keyframe, e.g. after a pipeline restart.
  void awaitKeyFrame();
  // Passthrough: the pipeline queue filled up; skip ahead to a requested keyframe.
  // Ignored while the first keyframe publishes, which stalls the queue by itself,
  // when such a publish started while this call waited for the publisher, and for
  // kBridgeQueueDuration after it ends, while its backlog drains.
  void onFellBehind(const std::function<void()> & request_key_frame);

private:
  friend class GStreamerStream;
  friend class RosStream;

  // May be invoked from ROS, GStreamer, or LiveKit worker threads after close() starts.
  void onSampleUnpackFailed(const std::string & error);
  void onCaptureFailed(const std::string & error);
  void onFellBehindFailed(const std::string & error);
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
  void logCaptureFailedLocked(const std::string & error);

  // The passthrough *Locked helpers return whether to request a keyframe; callers
  // do so after releasing mutex_ (see captureEncoded()).
  bool captureEncodedLocked(livekit::EncodedVideoSource::Frame frame);
  void logWaitingForFirstKeyFrameLocked();
  // Publishes the encoded source on first use; throws if the publish or send fails.
  void sendEncodedFrameLocked(livekit::EncodedVideoSource::Frame & frame);
  void awaitKeyFrameLocked();
  bool keyFrameRequestDueLocked();
  bool publishBackoffActiveLocked() const;
  void publishEncodedSourceLocked(std::uint32_t width, std::uint32_t height);
  bool handleEncodedFeedbackLocked(std::size_t frame_bytes, bool sent_key_frame);

  static constexpr auto kKeyFrameRequestInterval = std::chrono::milliseconds(500);
  static constexpr auto kPublishRetryInterval = std::chrono::seconds(1);
  // How much video the passthrough pipeline's bridge queue holds.
  static constexpr auto kBridgeQueueDuration = std::chrono::seconds(1);
  static constexpr auto kLogThrottle = std::chrono::seconds(5);

  RoomConnection & connection_;
  StreamSpec spec_;

  // Guards stream handles, publication state, and late callbacks racing with close().
  std::mutex mutex_;
  bool closed_ = false;
  bool published_once_ = false;
  bool captured_frame_logged_ = false;
  std::shared_ptr<RosStream> ros_stream_;
  std::unique_ptr<GStreamerStream> gstreamer_stream_;
  std::shared_ptr<livekit::VideoSource> source_;
  std::shared_ptr<livekit::LocalVideoTrack> track_;

  // Passthrough state.
  std::shared_ptr<livekit::EncodedVideoSource> encoded_source_;
  // Incremented under mutex_ when a publish starts and again when it ends, so it is
  // odd while the first keyframe publishes; read by onFellBehind() without mutex_.
  std::atomic<std::uint64_t> encoded_publish_sequence_{0U};
  // When the last publish ended, successful or not; min() before the first one.
  std::chrono::steady_clock::time_point encoded_publish_ended_at_ = std::chrono::steady_clock::time_point::min();
  bool awaiting_key_frame_ = true;
  // Set by a failed publish; keyframes before it are discarded so an unreachable
  // server doesn't get a publish attempt at every keyframe.
  std::chrono::steady_clock::time_point next_publish_attempt_{};
  std::size_t discarded_frame_count_ = 0U;
  EventThrottle key_frame_request_throttle_{kKeyFrameRequestInterval};
  // LiveKit asked for a keyframe that the throttle has not let through yet.
  bool key_frame_request_pending_ = false;
  EventThrottle fell_behind_log_throttle_{kLogThrottle};
  // A publish or send can fail at every keyframe.
  EventThrottle capture_failed_log_throttle_{kLogThrottle};
  // The queue overruns on every frame while it stays full.
  EventThrottle fell_behind_failed_log_throttle_{kLogThrottle};
  EventThrottle resumed_log_throttle_{kLogThrottle};
  EventThrottle waiting_log_throttle_{kLogThrottle};
  EventThrottle over_budget_log_throttle_{kLogThrottle};
  EventThrottle feedback_failed_log_throttle_{kLogThrottle};
  // A source that stays broken fails every restart; log it once per interval.
  EventThrottle restart_failed_log_throttle_{kLogThrottle};
  PassthroughBitrateMonitor bitrate_monitor_;
};

}  // namespace livekit_ros2_bridge::video
