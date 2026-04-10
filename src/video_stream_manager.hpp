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
#include "room_session.hpp"
#include "video_config.hpp"

namespace livekit_ros2_bridge
{

// Owns one in-process video runtime per resolved stream key.
class VideoStreamManager final
{
public:
  VideoStreamManager(rclcpp::Node & node, RoomSession & session);
  ~VideoStreamManager();

  std::string ensureStream(const SidecarLaunchSpec & spec);
  void stopStream(const std::string & stream_key);
  void shutdown();

private:
  class StreamRecord;

  rclcpp::Node & node_;
  RoomSession & session_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  std::unordered_map<std::string, std::shared_ptr<StreamRecord>> streams_;
};

}  // namespace livekit_ros2_bridge
