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

// Parses control packets and forwards commands to handlers. The router
// preserves caller-thread affinity and leaves any required executor handoff or
// synchronization to those handlers.  Payloads are passed by value so handlers can assume ownership
// without depending on packet storage lifetimes.
class ControlPacketRouter final
{
public:
  using HeartbeatHandler = std::function<void(std::string requester_identity, SubscriptionHeartbeat heartbeat)>;
  using PublishHandler = std::function<void(std::string requester_identity, TopicPublishCommand command)>;

  struct Handlers
  {
    HeartbeatHandler heartbeat_handler;
    PublishHandler publish_handler;
  };

  // todo: does it really need to inject the logger?
  ControlPacketRouter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, Handlers handlers);

  void route(const IncomingControlPacket & packet) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  HeartbeatHandler heartbeat_handler_;
  PublishHandler publish_handler_;
};

}  // namespace livekit_ros2_bridge
