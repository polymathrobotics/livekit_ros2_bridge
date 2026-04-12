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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/clock.hpp"

namespace livekit
{
class LocalDataTrack;
}  // namespace livekit

namespace livekit_ros2_bridge
{

class RoomSession;

// SubscriptionRegistry coordinates shared leases, DataStreamInstance owns each topic-level ROS
// data runtime, and DataTrackPublisher only owns the LiveKit data-track publications.
class DataTrackPublisher final
{
public:
  using PublishAcceptedFn = std::function<bool(std::size_t generation)>;
  using PublishFailedFn = std::function<void()>;

  DataTrackPublisher(RoomSession & session, std::string track_name, rclcpp::Clock::SharedPtr clock);

  // Best-effort push to an already-published data track carrying ROS CDR payloads. Missing
  // tracks and backpressure are dropped so ROS message delivery never blocks waiting on LiveKit's
  // data-track queue.
  void tryPush(const std::uint8_t * data, std::size_t size);
  // Publishes the LiveKit data track for a registry-reserved track name. Completion is reported
  // back with the caller's generation so the registry can reject stale publishes after teardown.
  void publish(
    std::size_t generation, const PublishAcceptedFn & publish_accepted_fn, const PublishFailedFn & publish_failed_fn);
  void unpublish();
  void shutdown();

private:
  RoomSession & session_;
  std::string track_name_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<livekit::LocalDataTrack> published_track_;
};

}  // namespace livekit_ros2_bridge
