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
#include "rclcpp/logging.hpp"
#include "ros_topic_publisher.hpp"
#include "subscription_lease_manager.hpp"
#include "utils/log_event.hpp"
#include "wire/protocol.hpp"
#include "wire/subscriptions.hpp"

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
  SubscriptionLeaseManager & subscription_lease_manager,
  RosTopicPublisher & ros_topic_publisher)
: clock_(std::move(clock))
, submit_to_executor_(std::move(submit_to_executor))
, subscription_lease_manager_(subscription_lease_manager)
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
  if (packet.topic == wire::protocol::kTopicPubTopic) {
    // Unlike heartbeats, publish requests have no session-based requester recovery path
    // downstream, so anonymous packets are rejected at the protocol boundary.
    if (packet.requester_identity.empty()) {
      LogEvent(kLogger, "packet_rejected")
        .field("reason", "missing_requester_identity")
        .fieldOr("requester_identity", packet.requester_identity)
        .warnThrottle(*clock_, kLogThrottle);
      return;
    }

    try {
      auto request = parseTopicPublishRequest(packet.payload);
      submitToExecutor([this, requester_identity = packet.requester_identity, request = std::move(request)]() mutable {
        ros_topic_publisher_.publish(requester_identity, request);
      });
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "packet_rejected")
        .field("reason", "invalid_publish_request")
        .fieldOr("requester_identity", packet.requester_identity)
        .field("error", exc.what())
        .warnThrottle(*clock_, kLogThrottle);
    }
  } else if (packet.topic == wire::protocol::kBridgeHeartbeatTopic) {
    try {
      nlohmann::json body = nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
      auto heartbeat = wire::subscriptions::parseHeartbeat(body);
      submitToExecutor(
        [this, requester_identity = packet.requester_identity, heartbeat = std::move(heartbeat)]() mutable {
          subscription_lease_manager_.handleHeartbeat(requester_identity, heartbeat);
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
