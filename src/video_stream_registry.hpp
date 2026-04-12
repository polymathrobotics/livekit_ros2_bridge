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
class VideoStreamRegistry final
{
public:
  VideoStreamRegistry(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    const SubscriptionQosConfig * subscription_qos_config = nullptr,
    VideoProfilingRegistry * profiling_registry = nullptr);
  ~VideoStreamRegistry();

  std::string ensureStreamRunning(const VideoStreamSpec & spec);
  void stopStream(const std::string & stream_key);
  void shutdown();

private:
  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  const SubscriptionQosConfig * subscription_qos_config_;
  VideoProfilingRegistry * profiling_registry_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  std::unordered_map<std::string, std::shared_ptr<VideoStreamInstance>> stream_instances_;
};

}  // namespace livekit_ros2_bridge
