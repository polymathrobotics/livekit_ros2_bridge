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

#include <functional>
#include <string>

#include "payloads/stream_control_payloads.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "room_connection.hpp"
#include "topic_publish_command.hpp"

namespace livekit_ros2_bridge
{

// Parses ingress control packets at the room-transport boundary and forwards only validated
// commands to runtime-specific handlers. The router preserves caller-thread affinity and leaves any
// required executor handoff or synchronization to those handlers.
class ControlPacketRouter final
{
public:
  // route() invokes handlers inline on the caller's thread after the matched control topic has
  // been fully parsed and validated. Parsed payloads are passed by value so handlers can assume
  // ownership without depending on packet storage lifetimes.
  using HeartbeatHandler = std::function<void(std::string requester_identity, SubscriptionHeartbeat heartbeat)>;
  using PublishHandler = std::function<void(std::string requester_identity, TopicPublishCommand command)>;

  struct Handlers
  {
    // Heartbeats may be dispatched with an empty requester_identity so downstream session-based
    // recovery can resolve the sender from heartbeat.session_id.
    HeartbeatHandler heartbeat_handler;
    // Publish commands are dispatched only after requester_identity presence has been enforced.
    PublishHandler publish_handler;
  };

  // The clock is used for throttled drop/rejection logs; all dependencies are required.
  ControlPacketRouter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, Handlers handlers);

  // Routes supported control topics synchronously. Unsupported topics, malformed payloads, and
  // std::exception failures from parsing or handlers are converted into throttled logs instead of
  // escaping the ingress path. Anonymous heartbeats are still forwarded so downstream session
  // fallback can recover requester_identity; publish commands are not.
  void route(const IncomingControlPacket & packet) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  HeartbeatHandler heartbeat_handler_;
  PublishHandler publish_handler_;
};

}  // namespace livekit_ros2_bridge
