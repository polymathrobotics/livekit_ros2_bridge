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
#include <functional>
#include <optional>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"

namespace livekit_ros2_bridge
{
namespace
{

template <typename Request, typename ParseRequest>
void parseAndRouteControlPacket(
  const rclcpp::Logger & logger,
  const IncomingControlPacket & packet,
  ParseRequest && parse_request,
  const std::function<void(std::string requester_identity, Request request)> & handler,
  bool require_requester_identity = true)
{
  if (require_requester_identity && packet.requester_identity.empty()) {
    RCLCPP_WARN(logger, "Ignoring %s packet without requester identity", packet.control_topic.c_str());
    return;
  }

  if (!handler) {
    return;
  }

  std::optional<Request> request = parse_request();
  if (!request) {
    return;
  }

  handler(packet.requester_identity, std::move(*request));
}

std::optional<SubscriptionHeartbeat> parseHeartbeatPayload(
  const rclcpp::Logger & logger, const IncomingControlPacket & packet)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      logger,
      "Ignoring malformed heartbeat from requester_identity=%s: %s",
      packet.requester_identity.c_str(),
      exc.what());
    return std::nullopt;
  }

  try {
    return parseSubscriptionHeartbeat(body);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      logger,
      "Ignoring invalid heartbeat from requester_identity=%s: %s",
      packet.requester_identity.c_str(),
      exc.what());
    return std::nullopt;
  }
}

std::optional<TopicPublishCommand> parseTopicPublishPayload(
  const rclcpp::Logger & logger, const IncomingControlPacket & packet)
{
  try {
    return parseTopicPublishCommand(packet.payload);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      logger,
      "Ignoring invalid publish command from requester_identity=%s: %s",
      packet.requester_identity.c_str(),
      exc.what());
    return std::nullopt;
  }
}

}  // namespace

ControlPacketRouter::ControlPacketRouter(rclcpp::Logger logger, Handlers handlers)
: logger_(std::move(logger))
, handlers_(std::move(handlers))
{}

void ControlPacketRouter::route(const IncomingControlPacket & packet) const
{
  // Unknown topics are a silent no-op so newer peers can add control packets without flooding logs
  // on bridges that only understand the current heartbeat and topic-publish vocabulary.
  if (packet.control_topic == protocol::kControlSubscriptionsHeartbeat) {
    // Heartbeats may arrive without requester_identity when LiveKit omits it from user data. The
    // heartbeat processor can still recover that identity from a leased session_id.
    parseAndRouteControlPacket<SubscriptionHeartbeat>(
      logger_,
      packet,
      [this, &packet]() { return parseHeartbeatPayload(logger_, packet); },
      handlers_.on_subscription_heartbeat,
      false);
    return;
  }

  if (packet.control_topic == protocol::kControlTopicPublish) {
    parseAndRouteControlPacket<TopicPublishCommand>(
      logger_,
      packet,
      [this, &packet]() { return parseTopicPublishPayload(logger_, packet); },
      handlers_.on_topic_publish_command);
  }
}

}  // namespace livekit_ros2_bridge
