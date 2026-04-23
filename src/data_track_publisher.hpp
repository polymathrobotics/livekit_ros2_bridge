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

#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;
class RoomConnection;

// Owns one deterministic LiveKit data-track publication backed by one ROS subscription.
class DataTrackPublisher final
{
public:
  static std::shared_ptr<DataTrackPublisher> create(
    std::string topic,
    std::string interface_type,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    RoomConnection & room_connection,
    const SubscriptionQosConfig * qos_config);

  ~DataTrackPublisher();

  DataTrackPublisher(const DataTrackPublisher &) = delete;
  DataTrackPublisher & operator=(const DataTrackPublisher &) = delete;
  DataTrackPublisher(DataTrackPublisher &&) = delete;
  DataTrackPublisher & operator=(DataTrackPublisher &&) = delete;

  void publish();
  int intervalMs() const;
  bool isPublished() const;
  void republish();
  void setIntervalMs(int interval_ms);
  const std::string & name() const;

private:
  class Publication;

  DataTrackPublisher(
    std::string topic,
    std::string interface_type,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    RoomConnection & room_connection,
    const SubscriptionQosConfig * qos_config);

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  rclcpp::Clock::SharedPtr clock_;
  RoomConnection & room_connection_;
  const SubscriptionQosConfig * qos_config_;

  std::string topic_;
  std::string interface_type_;
  std::string track_name_;

  int interval_ms_ = 0;
  bool publish_failed_ = false;
  std::unique_ptr<Publication> publication_;
};

}  // namespace livekit_ros2_bridge
