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

#include "control_packet_router.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLogThrottle = std::chrono::seconds(5);
}  // namespace

ControlPacketRouter::ControlPacketRouter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, Handlers handlers)
: logger_(std::move(logger))
, clock_(std::move(clock))
, heartbeat_handler_(std::move(handlers.heartbeat_handler))
, publish_handler_(std::move(handlers.publish_handler))
{
  if (clock_ == nullptr) {
    throw std::invalid_argument("ControlPacketRouter requires a clock.");
  }

  if (!heartbeat_handler_) {
    throw std::invalid_argument("ControlPacketRouter requires a heartbeat callback.");
  }

  if (!publish_handler_) {
    throw std::invalid_argument("ControlPacketRouter requires a publish callback.");
  }
}

void ControlPacketRouter::route(const IncomingControlPacket & packet) const
{
  // todo: extract 2 route functions
  if (packet.control_topic == protocol::kControlTopicPublish) {
    bool missing_requester_identity = false;
    try {
      // Unlike heartbeats, publish commands have no session-based requester recovery path
      // downstream, so anonymous packets are rejected at the protocol boundary.
      if (packet.requester_identity.empty()) {
        missing_requester_identity = true;
        throw std::invalid_argument("requester_identity is required");
      }
      publish_handler_(packet.requester_identity, parseTopicPublishCommand(packet.payload));
    } catch (const std::exception & exc) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", missing_requester_identity ? "missing_requester_identity" : "invalid_publish_command")
        .fieldOr("requester_identity", packet.requester_identity)
        .fieldIf(!missing_requester_identity, "error", exc.what())
        .warnThrottle(*clock_, kLogThrottle);
    }
  } else if (packet.control_topic == protocol::kControlSubscriptionsHeartbeat) {
    try {
      nlohmann::json body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
      heartbeat_handler_(packet.requester_identity, stream_control_payloads::parseSubscriptionHeartbeat(body));
    } catch (const std::exception & exc) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", "invalid_heartbeat")
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, kLogThrottle);
    }
  } else {
    LogEvent(logger_, "control_packet_dropped")
      .field("reason", "unsupported_control_topic")
      .field("control_topic", packet.control_topic)
      .fieldOr("requester_identity", packet.requester_identity)
      .warnThrottle(*clock_, kLogThrottle);
  }
}

}  // namespace livekit_ros2_bridge
