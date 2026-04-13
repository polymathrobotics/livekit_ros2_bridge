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

class RoomConnection;

// SubscriptionRegistry coordinates shared leases, DataStreamInstance owns each topic-level ROS
// data runtime, and DataTrackPublisher only owns the LiveKit data-track publications.
class DataTrackPublisher final
{
public:
  // Called after LiveKit returns a new track but before this publisher adopts it. Returning false
  // rejects that publish generation, reclaims the just-published track, and leaves any previously
  // active track untouched.
  using AcceptHandler = std::function<bool(std::size_t generation)>;

  // Called when a publish attempt cannot produce an active track for this publisher, including
  // publish exceptions and exceptions escaping AcceptHandler. A false AcceptHandler result is
  // treated as a stale completion rather than a failure signal.
  using FailHandler = std::function<void()>;

  DataTrackPublisher(RoomConnection & connection, std::string track_name, rclcpp::Clock::SharedPtr clock);

  // Best-effort write to an already-published data track carrying ROS CDR payloads. Missing
  // tracks, backpressure, and transient LiveKit push failures are swallowed so ROS message
  // delivery never blocks or tears down the current publication.
  void write(const std::uint8_t * cdr, std::size_t size);

  // Publishes the LiveKit data track for a registry-reserved track name. Callbacks run inline,
  // and the generation lets the registry reject stale completions after teardown or same-topic
  // replacement.
  void publish(std::size_t generation, const AcceptHandler & on_accept, const FailHandler & on_fail);

  // Best-effort teardown of the currently accepted publication. After this returns, later writes
  // are ignored even if LiveKit rejected the unpublish request.
  void unpublish();

private:
  // Non-owning room connection facade. The connection must outlive this publisher.
  RoomConnection & room_connection_;
  std::string track_name_;
  rclcpp::Clock::SharedPtr log_clock_;
  // Only the currently accepted publication lives here. Rejected or failed replacement attempts
  // are reclaimed before this handle changes, and unpublish() clears it before touching LiveKit.
  std::shared_ptr<livekit::LocalDataTrack> published_track_;
};

}  // namespace livekit_ros2_bridge
