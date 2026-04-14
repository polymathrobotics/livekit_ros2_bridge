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

#include "packet_router.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

#include "nlohmann/json.hpp"
#include "payloads/stream_control_payloads.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "ros_topic_publisher.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLogThrottle = std::chrono::seconds(5);
const auto kLogger = rclcpp::get_logger("packet_router");
}  // namespace

PacketRouter::PacketRouter(
  rclcpp::Clock::SharedPtr clock,
  SubmitToExecutorFunction submit_to_executor,
  SubscriptionHeartbeatProcessor & subscription_heartbeat_processor,
  RosTopicPublisher & ros_topic_publisher)
: clock_(std::move(clock))
, submit_to_executor_(std::move(submit_to_executor))
, subscription_heartbeat_processor_(subscription_heartbeat_processor)
, ros_topic_publisher_(ros_topic_publisher)
{
  if (clock_ == nullptr) {
    throw std::invalid_argument("PacketRouter requires a clock.");
  }

  if (!submit_to_executor_) {
    throw std::invalid_argument("PacketRouter requires a submitToExecutor callback.");
  }
}

void PacketRouter::handle(const IncomingPacket & packet) const
{
  if (packet.topic == wire::protocol::kControlTopicPublish) {
    bool missing_requester_identity = false;
    try {
      // Unlike heartbeats, publish commands have no session-based requester recovery path
      // downstream, so anonymous packets are rejected at the protocol boundary.
      if (packet.requester_identity.empty()) {
        missing_requester_identity = true;
        throw std::invalid_argument("requester_identity is required");
      }
      auto command = parseTopicPublishCommand(packet.payload);
      submitToExecutor([this, requester_identity = packet.requester_identity, command = std::move(command)]() mutable {
        ros_topic_publisher_.publish(requester_identity, command);
      });
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "packet_rejected")
        .field("reason", missing_requester_identity ? "missing_requester_identity" : "invalid_publish_command")
        .fieldOr("requester_identity", packet.requester_identity)
        .fieldIf(!missing_requester_identity, "error", exc.what())
        .warnThrottle(*clock_, kLogThrottle);
    }
  } else if (packet.topic == wire::protocol::kControlSubscriptionsHeartbeat) {
    try {
      nlohmann::json body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
      auto heartbeat = wire::subscriptions::parseSubscriptionHeartbeat(body);
      submitToExecutor(
        [this, requester_identity = packet.requester_identity, heartbeat = std::move(heartbeat)]() mutable {
          subscription_heartbeat_processor_.process(requester_identity, heartbeat);
        });
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "packet_rejected")
        .field("reason", "invalid_heartbeat")
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, kLogThrottle);
    }
  } else {
    LogEvent(kLogger, "packet_dropped")
      .field("reason", "unsupported_topic")
      .field("topic", packet.topic)
      .fieldOr("requester_identity", packet.requester_identity)
      .warnThrottle(*clock_, kLogThrottle);
  }
}

void PacketRouter::submitToExecutor(std::function<void()> work) const
{
  submit_to_executor_(std::move(work));
}

}  // namespace livekit_ros2_bridge
