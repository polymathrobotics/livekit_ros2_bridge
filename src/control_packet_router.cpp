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

constexpr auto kControlPacketLogThrottleMs = 5000;
}  // namespace

ControlPacketRouter::ControlPacketRouter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, Handlers handlers)
: logger_(std::move(logger))
, clock_(std::move(clock))
, handlers_(std::move(handlers))
{
  if (clock_ == nullptr) {
    throw std::invalid_argument("ControlPacketRouter requires a clock.");
  }

  if (!handlers_.on_subscription_heartbeat) {
    throw std::invalid_argument("ControlPacketRouter requires an on_subscription_heartbeat handler.");
  }

  if (!handlers_.on_topic_publish_command) {
    throw std::invalid_argument("ControlPacketRouter requires an on_topic_publish_command handler.");
  }
}

void ControlPacketRouter::route(const IncomingControlPacket & packet) const
{
  const auto rejection_log_interval = std::chrono::milliseconds(kControlPacketLogThrottleMs);
  if (packet.control_topic == protocol::kControlSubscriptionsHeartbeat) {
    // Heartbeats may arrive without requester_identity when LiveKit omits it from user data. The
    // heartbeat processor can still recover that identity from a leased session_id.
    nlohmann::json body;
    try {
      body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
    } catch (const std::exception & exc) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", "malformed_heartbeat")
        .field("control_topic", packet.control_topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, rejection_log_interval);
      return;
    }

    try {
      handlers_.on_subscription_heartbeat(packet.requester_identity, parseSubscriptionHeartbeat(body));
    } catch (const std::exception & exc) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", "invalid_heartbeat")
        .field("control_topic", packet.control_topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, rejection_log_interval);
    }
    return;
  }

  if (packet.control_topic == protocol::kControlTopicPublish) {
    if (packet.requester_identity.empty()) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", "missing_requester_identity")
        .field("control_topic", packet.control_topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .warnThrottle(*clock_, rejection_log_interval);
      return;
    }

    try {
      handlers_.on_topic_publish_command(packet.requester_identity, parseTopicPublishCommand(packet.payload));
    } catch (const std::exception & exc) {
      LogEvent(logger_, "control_packet_rejected")
        .field("reason", "invalid_publish_command")
        .field("control_topic", packet.control_topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, rejection_log_interval);
    }
    return;
  }

  LogEvent(logger_, "control_packet_dropped")
    .field("reason", "unsupported_control_topic")
    .field("control_topic", packet.control_topic)
    .fieldOr("requester_identity", packet.requester_identity)
    .warnThrottle(*clock_, std::chrono::milliseconds(kControlPacketLogThrottleMs));
}

}  // namespace livekit_ros2_bridge
