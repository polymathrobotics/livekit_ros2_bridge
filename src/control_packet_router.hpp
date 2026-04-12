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

class ControlPacketRouter final
{
public:
  struct Callbacks
  {
    std::function<void(std::string requester_identity, SubscriptionHeartbeat heartbeat)> on_subscription_heartbeat;
    std::function<void(std::string requester_identity, TopicPublishCommand command)> on_topic_publish_command;
  };

  // Both callbacks are required; construction fails if either callback is empty.
  ControlPacketRouter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, Callbacks callbacks);

  // Routes only the control topics this bridge understands. Unknown topics, malformed payloads,
  // and anonymous publish commands are dropped before dispatch. Anonymous heartbeats are still
  // forwarded so client-session fallback can recover the requester identity downstream.
  void route(const IncomingControlPacket & packet) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Callbacks callbacks_;
};

}  // namespace livekit_ros2_bridge
