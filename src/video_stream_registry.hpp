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

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "core/subscriptions.hpp"
#include "video_stream_config.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace livekit_ros2_bridge
{

class RoomConnection;
struct SubscriptionQosConfig;
class VideoProfilingRegistry;
struct VideoStreamSpec;
class VideoStreamInstance;

// Stable video metadata derived from one resolved subscription target and surfaced back to callers for
// status and logging.
struct VideoStreamInfo
{
  std::string stream_key;
  std::string track_name;
  std::string degraded_reason;
};

// Registry of shared in-process video runtimes keyed by resolved stream key.
// Public methods are thread-safe. Once shutdown begins, later start requests fail.
class VideoStreamRegistry final
{
public:
  // All dependencies are borrowed. `node` and `room_connection` must outlive the registry;
  // `qos_config` must outlive any stream instances created from it;
  // `profiling_registry` is only consulted while creating new instances.
  VideoStreamRegistry(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    const SubscriptionQosConfig * qos_config = nullptr,
    VideoProfilingRegistry * profiling_registry = nullptr,
    const VideoStreamConfig * video_stream_config = nullptr);
  ~VideoStreamRegistry();

  // Resolves one subscription-facing target into the stable shared runtime identity used for
  // starts, status, and teardown.
  VideoStreamInfo resolve(
    SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type = "") const;
  // Returns metadata only while the resolved stream currently has a live registry entry.
  std::optional<VideoStreamInfo> find(
    SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type = "") const;
  // Starts or reuses the shared runtime addressed by these subscription-facing fields.
  void start(SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type = "");
  // Detaches the current runtime selected by these fields if present. A later `start()` creates
  // a fresh instance instead of reusing one that is already shutting down.
  void stop(SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type = "");
  // Idempotent terminal teardown. Instances are detached under the mutex and shut down after
  // unlocking because teardown touches external ROS and LiveKit state.
  void shutdown();

private:
  VideoStreamSpec resolveSpec(
    SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const;

  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  // Optional non-owning QoS config forwarded into new stream instances.
  const SubscriptionQosConfig * qos_config_;
  // Optional non-owning registry consulted only when creating a new stream instance.
  VideoProfilingRegistry * profiling_registry_;
  // Optional non-owning config used to resolve topic/configured-source requests.
  VideoStreamConfig default_video_stream_config_;
  const VideoStreamConfig * video_stream_config_;
  // Guards `is_shutdown_` and `instances_`. Per-stream lifecycle synchronization lives
  // inside each VideoStreamInstance so the registry only serializes map membership and terminal
  // shutdown.
  mutable std::mutex mutex_;
  bool is_shutdown_ = false;
  // Shared ownership keeps an instance alive after the registry lock is released so callers can
  // start or stop it without holding the mutex.
  std::unordered_map<std::string, std::shared_ptr<VideoStreamInstance>> instances_;
};

}  // namespace livekit_ros2_bridge
