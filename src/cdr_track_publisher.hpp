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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/clock.hpp"

namespace livekit
{
class LocalDataTrack;
}  // namespace livekit

namespace livekit_ros2_bridge
{

class RoomSession;
class SubscriptionRegistry;

class CdrTrackPublisher final
{
public:
  CdrTrackPublisher(RoomSession & session, rclcpp::Clock::SharedPtr clock);

  // Best-effort push to an already-published CDR data track. Missing tracks and backpressure are
  // dropped so ROS message delivery never blocks waiting on LiveKit's data-track queue.
  void pushMessage(const std::string & track_name, const std::uint8_t * data, std::size_t size);
  // Publishes the LiveKit data track for a registry-reserved track name. Completion is reported
  // back with the caller's generation so the registry can reject stale publishes after teardown.
  void publishTrack(
    const std::string & track_name, std::size_t generation, SubscriptionRegistry & subscription_registry);
  void unpublishTrack(const std::string & track_name);
  void unpublishAll();

private:
  RoomSession & session_;
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, std::shared_ptr<livekit::LocalDataTrack>> published_tracks_;
};

}  // namespace livekit_ros2_bridge
