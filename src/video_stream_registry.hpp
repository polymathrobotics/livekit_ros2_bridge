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
#include <string>
#include <unordered_map>

#include "rclcpp/node.hpp"
#include "room_connection.hpp"
#include "subscription_qos.hpp"
#include "video_profiling.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

class VideoStreamInstance;

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
    VideoProfilingRegistry * profiling_registry = nullptr);
  ~VideoStreamRegistry();

  // Returns the LiveKit track name for the shared runtime selected by `spec.stream_key`. Once a
  // key exists, later starts reuse that instance and track identity.
  std::string start(const VideoStreamSpec & spec);
  // Detaches the current runtime for `stream_key` if present. A later `start()` creates a fresh
  // instance instead of reusing one that is already shutting down.
  void stop(const std::string & stream_key);
  // Idempotent terminal teardown. Instances are detached under the mutex and shut down after
  // unlocking because teardown touches external ROS and LiveKit state.
  void shutdown();

private:
  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  // Optional non-owning QoS config forwarded into new stream instances.
  const SubscriptionQosConfig * qos_config_;
  // Optional non-owning registry consulted only when creating a new stream instance.
  VideoProfilingRegistry * profiling_registry_;
  // Guards `is_shutdown_` and `instances_`. Per-stream lifecycle synchronization lives
  // inside each VideoStreamInstance so the registry only serializes map membership and terminal
  // shutdown.
  std::mutex mutex_;
  bool is_shutdown_ = false;
  // Shared ownership keeps an instance alive after the registry lock is released so callers can
  // start or stop it without holding the mutex.
  std::unordered_map<std::string, std::shared_ptr<VideoStreamInstance>> instances_;
};

}  // namespace livekit_ros2_bridge
