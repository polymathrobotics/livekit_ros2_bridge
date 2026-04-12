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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "data_track_publisher.hpp"

namespace rclcpp
{
class GenericSubscription;
class Node;
class SerializedMessage;
}  // namespace rclcpp

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;
class QuiesceGate;
class RoomConnection;
class SubscriptionRegistry;

// SubscriptionRegistry owns the shared lease state for a topic and creates one DataStreamInstance
// when that topic needs a data delivery runtime. Each DataStreamInstance owns the ROS
// subscription plus one DataTrackPublisher for the matching LiveKit data track.
class DataStreamInstance final : public std::enable_shared_from_this<DataStreamInstance>
{
public:
  using Clock = std::chrono::steady_clock;

  enum class State
  {
    kNone,
    kPending,
    kPublished,
    kFailed
  };

  static std::shared_ptr<DataStreamInstance> create(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    SubscriptionRegistry & subscription_registry,
    std::string topic,
    std::string interface_type,
    int applied_interval_ms,
    std::size_t publish_generation,
    QuiesceGate & message_callback_gate,
    const SubscriptionQosConfig * subscription_qos_config = nullptr);

  DataStreamInstance(const DataStreamInstance &) = delete;
  DataStreamInstance & operator=(const DataStreamInstance &) = delete;
  DataStreamInstance(DataStreamInstance &&) = delete;
  DataStreamInstance & operator=(DataStreamInstance &&) = delete;
  ~DataStreamInstance() = default;

  const std::string & trackName() const;
  int appliedIntervalMs() const;
  State state() const;

  void updateAppliedIntervalMs(int applied_interval_ms);
  void start(const std::string & requester_identity);
  void republish(const std::string & requester_identity);
  bool onPublishComplete(std::size_t generation);
  void onPublishFailed();
  void shutdown();

private:
  DataStreamInstance(
    rclcpp::Node & node,
    RoomConnection & room_connection,
    SubscriptionRegistry & subscription_registry,
    std::string topic,
    std::string interface_type,
    int applied_interval_ms,
    std::size_t publish_generation,
    QuiesceGate & message_callback_gate,
    const SubscriptionQosConfig * subscription_qos_config);

  void initializeSubscription();
  void handleSerializedMessage(const rclcpp::SerializedMessage & message);
  bool shouldSkipDueToInterval();
  void publishPendingDataTrack(const std::string & requester_identity);
  static std::string deriveTrackName(const std::string & normalized_topic);

  rclcpp::Node & node_;
  std::string topic_;
  std::string interface_type_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_handle_;
  std::optional<Clock::time_point> last_sent_time_;
  std::string track_name_;
  DataTrackPublisher data_track_publisher_;
  int applied_interval_ms_ = 0;
  State state_ = State::kNone;
  std::size_t generation_ = 0U;
  std::size_t callback_generation_ = 0U;
  SubscriptionRegistry & subscription_registry_;
  QuiesceGate & message_callback_gate_;
  const SubscriptionQosConfig * subscription_qos_config_;
};

}  // namespace livekit_ros2_bridge
