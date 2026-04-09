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

namespace livekit_ros2_bridge
{
ControlPacketRouter::ControlPacketRouter(rclcpp::Logger logger, Handlers handlers)
: logger_(std::move(logger))
, handlers_(std::move(handlers))
{
  if (!handlers_.on_subscription_heartbeat) {
    throw std::invalid_argument("ControlPacketRouter requires an on_subscription_heartbeat handler.");
  }

  if (!handlers_.on_topic_publish_command) {
    throw std::invalid_argument("ControlPacketRouter requires an on_topic_publish_command handler.");
  }
}

void ControlPacketRouter::route(const IncomingControlPacket & packet) const
{
  // Unknown topics are a silent no-op so newer peers can add control packets without flooding logs
  // on bridges that only understand the current heartbeat and topic-publish vocabulary.
  if (packet.control_topic == protocol::kControlSubscriptionsHeartbeat) {
    // Heartbeats may arrive without requester_identity when LiveKit omits it from user data. The
    // heartbeat processor can still recover that identity from a leased session_id.
    nlohmann::json body;
    try {
      body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        logger_,
        "event=control_packet_rejected reason=malformed_heartbeat control_topic=%s requester_identity=%s "
        "error=%s",
        packet.control_topic.c_str(),
        packet.requester_identity.c_str(),
        exc.what());
      return;
    }

    try {
      handlers_.on_subscription_heartbeat(packet.requester_identity, parseSubscriptionHeartbeat(body));
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        logger_,
        "event=control_packet_rejected reason=invalid_heartbeat control_topic=%s requester_identity=%s "
        "error=%s",
        packet.control_topic.c_str(),
        packet.requester_identity.c_str(),
        exc.what());
    }
    return;
  }

  if (packet.control_topic == protocol::kControlTopicPublish) {
    if (packet.requester_identity.empty()) {
      RCLCPP_WARN(
        logger_,
        "event=control_packet_rejected reason=missing_requester_identity control_topic=%s",
        packet.control_topic.c_str());
      return;
    }

    try {
      handlers_.on_topic_publish_command(packet.requester_identity, parseTopicPublishCommand(packet.payload));
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        logger_,
        "event=control_packet_rejected reason=invalid_publish_command control_topic=%s requester_identity=%s "
        "error=%s",
        packet.control_topic.c_str(),
        packet.requester_identity.c_str(),
        exc.what());
    }
  }
}

}  // namespace livekit_ros2_bridge
