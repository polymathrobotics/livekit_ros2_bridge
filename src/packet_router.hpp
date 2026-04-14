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

#include "rclcpp/clock.hpp"
#include "room_connection.hpp"

namespace livekit_ros2_bridge
{

class RosTopicPublisher;
class SubscriptionLeaseManager;

// Parses incoming room packets and dispatches the supported packet topics through the ROS-facing
// runtime helpers.
class PacketRouter final
{
public:
  using SubmitToExecutorFunction = std::function<void(std::function<void()> work)>;

  PacketRouter(
    rclcpp::Clock::SharedPtr clock,
    SubmitToExecutorFunction submit_to_executor,
    SubscriptionLeaseManager & subscription_lease_manager,
    RosTopicPublisher & ros_topic_publisher);

  void handle(const IncomingPacket & packet) const;

private:
  void submitToExecutor(std::function<void()> work) const;

  rclcpp::Clock::SharedPtr clock_;

  // RoomConnection may invoke packet callbacks from its own worker threads.
  // PacketRouter parses and dispatches immediately, but ROS-visible side
  // effects are bounced through submitToExecutor() in the runtime so they stay
  // serialized with shutdown and other session-state mutations on the executor
  // queue.
  SubmitToExecutorFunction submit_to_executor_;
  SubscriptionLeaseManager & subscription_lease_manager_;
  RosTopicPublisher & ros_topic_publisher_;
};

}  // namespace livekit_ros2_bridge
