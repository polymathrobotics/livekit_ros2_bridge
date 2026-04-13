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

constexpr auto kLogThrottleMs = 5000;
constexpr auto kLogThrottle = std::chrono::milliseconds(kLogThrottleMs);
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
  if (packet.control_topic == protocol::kControlSubscriptionsHeartbeat) {
    // Keep transport-level JSON decoding separate from semantic heartbeat validation so the
    // rejection reason distinguishes broken wire payloads from protocol-shape violations.
    nlohmann::json body;
    try {
      body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
    } catch (const std::exception & exc) {
      logRejection(packet, "malformed_heartbeat", exc.what());
      return;
    }

    try {
      heartbeat_handler_(packet.requester_identity, stream_control_payloads::parseSubscriptionHeartbeat(body));
    } catch (const std::exception & exc) {
      logRejection(packet, "invalid_heartbeat", exc.what());
    }
    return;
  }

  if (packet.control_topic == protocol::kControlTopicPublish) {
    // Unlike heartbeats, publish commands have no session-based requester recovery path
    // downstream, so anonymous packets are rejected at the protocol boundary.
    if (packet.requester_identity.empty()) {
      logRejection(packet, "missing_requester_identity");
      return;
    }

    try {
      publish_handler_(packet.requester_identity, parseTopicPublishCommand(packet.payload));
    } catch (const std::exception & exc) {
      logRejection(packet, "invalid_publish_command", exc.what());
    }
    return;
  }

  LogEvent(logger_, "control_packet_dropped")
    .field("reason", "unsupported_control_topic")
    .field("control_topic", packet.control_topic)
    .fieldOr("requester_identity", packet.requester_identity)
    .warnThrottle(*clock_, kLogThrottle);
}

void ControlPacketRouter::logRejection(
  const IncomingControlPacket & packet, const char * reason, const char * error) const
{
  LogEvent event(logger_, "control_packet_rejected");
  event.field("reason", reason).fieldOr("requester_identity", packet.requester_identity);
  if (error != nullptr) {
    event.field("error", error);
  }
  event.warnThrottle(*clock_, kLogThrottle);
}

}  // namespace livekit_ros2_bridge
