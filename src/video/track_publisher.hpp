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

  // Returns the camera's bitrate when a window of a few seconds averaged well above the target.
  std::optional<std::uint64_t> record(std::size_t frame_bytes, Clock::time_point now);

private:
  std::uint64_t target_bitrate_bps_ = 0U;
  Clock::time_point window_started_at_{};
  std::uint64_t window_bytes_ = 0U;
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

  // The seam for tests: a real source can't be driven without a room.
  using EncodedFeedbackReader =
    std::function<livekit::EncodedVideoSource::Feedback(const livekit::EncodedVideoSource & source)>;

  // The seam for tests: lets them advance time instead of sleeping.
  using NowProvider = std::function<std::chrono::steady_clock::time_point()>;

  // Does not start a ROS or GStreamer input stream.
  TrackPublisher(
    RoomConnection & connection,
    StreamSpec spec,
    EncodedFeedbackReader read_encoded_feedback = nullptr,
    NowProvider now_provider = nullptr);

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

  // Passthrough: publishes on the first keyframe, discarding frames until then or after a gap.
  // Also passes on LiveKit's keyframe requests. A failed send is discarded; a failed publish, or a
  // published track still without a keyframe after the stall timeout, throws.
  // Returns whether the frame was sent.
  bool captureEncoded(
    livekit::EncodedVideoSource::Frame frame,
    const std::function<void()> & request_key_frame,
    bool discontinuous = false);
  void awaitKeyFrame();

private:
  friend class GStreamerStream;
  friend class RosStream;

  // May be invoked from ROS, GStreamer, or LiveKit worker threads after close() starts.
  void onSampleUnpackFailed(const std::string & error);
  void onCaptureFailed(const std::string & error);
  void onKeyFrameWithoutParameterSets();
  void onRestartFailed(const std::string & error);
  void onAppsrcPushFailed(const std::string & error);

  PipelineCallbacks makePipelineCallbacks(
    std::function<bool()> is_shutdown, std::function<void(const std::string & reason)> on_failed);
  PipelineCallbacks makeEncodedPipelineCallbacks(
    std::function<bool()> is_shutdown,
    std::function<void(const std::string & reason)> on_failed,
    std::function<void(const std::string & reason, const std::string & offered_caps)> on_caps_mismatch,
    std::function<void()> request_key_frame);

  void close();

  bool captureEncodedLocked(livekit::EncodedVideoSource::Frame frame, bool discontinuous);
  void logWaitingForFirstKeyFrameLocked();
  bool sendEncodedFrameLocked(livekit::EncodedVideoSource::Frame & frame);
  void awaitKeyFrameLocked();
  bool keyFrameRequestDueLocked();
  void checkKeyFrameStallLocked();
  void publishEncodedSourceLocked(std::uint32_t width, std::uint32_t height);
  livekit::EncodedVideoSource::Feedback readEncodedFeedbackLocked();
  void checkEncodedBitrateLocked(const livekit::EncodedVideoSource::Feedback & feedback, std::size_t frame_bytes);
  void logResumedLocked();
  void logDiscontinuityLocked();
  void logViewerKeyFrameRequestLocked();
  void logThrottledFailureLocked(EventThrottle & throttle, const char * event, const std::string & error);

  static constexpr auto kKeyFrameRequestInterval = std::chrono::milliseconds(500);
  // How long a published track may wait for a keyframe before the pipeline restarts.
  static constexpr auto kKeyFrameStallTimeout = std::chrono::seconds(5);
  // A restart of a source with a long keyframe interval can need longer than the base to get one.
  static constexpr auto kMaxKeyFrameStallTimeout = std::chrono::seconds(30);
  static constexpr auto kLogThrottle = std::chrono::seconds(5);

  RoomConnection & connection_;
  StreamSpec spec_;
  EncodedFeedbackReader read_encoded_feedback_;
  NowProvider now_provider_;
  // Doubles after each stall restart and persists across them; a sent keyframe resets it.
  std::chrono::milliseconds current_key_frame_stall_timeout_;

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

  std::shared_ptr<livekit::EncodedVideoSource> encoded_source_;
  bool awaiting_key_frame_ = true;
  // Only set while awaiting_key_frame_, by the first frame the wait discards or fails, so a slow
  // camera start doesn't count.
  std::optional<std::chrono::steady_clock::time_point> awaiting_since_;
  // Set by a publish, which stalls the sample thread, so a gap right after it is expected.
  bool just_published_ = false;
  // LiveKit asks once and then drops delta frames until a keyframe arrives, so keep asking the source.
  bool livekit_key_frame_request_pending_ = false;
  // Warn once: the cause is the source's setup, and the publisher outlives restarts.
  bool parameter_sets_warning_logged_ = false;
  std::size_t discarded_frame_count_ = 0U;
  EventThrottle key_frame_request_throttle_{kKeyFrameRequestInterval};
  EventThrottle discontinuity_log_throttle_{kLogThrottle};
  EventThrottle resumed_log_throttle_{kLogThrottle};
  EventThrottle waiting_log_throttle_{kLogThrottle};
  EventThrottle viewer_key_frame_request_log_throttle_{kLogThrottle};
  EventThrottle feedback_failed_log_throttle_{kLogThrottle};
  EventThrottle send_failed_log_throttle_{kLogThrottle};
  PassthroughBitrateMonitor bitrate_monitor_;
};

}  // namespace livekit_ros2_bridge::video
