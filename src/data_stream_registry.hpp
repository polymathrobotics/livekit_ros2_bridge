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

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include "data_stream_instance.hpp"
#include "rclcpp/node.hpp"
#include "utils/quiesce_gate.hpp"

namespace livekit_ros2_bridge
{

class RoomConnection;
struct SubscriptionQosConfig;

// Owns one data delivery runtime per canonical topic and the callback bookkeeping needed to
// safely recycle deterministic track names across reset and teardown.
class DataStreamRegistry final
{
public:
  DataStreamRegistry(
    rclcpp::Node & node, RoomConnection & room_connection, const SubscriptionQosConfig * qos_config = nullptr);
  ~DataStreamRegistry();

  void create(const std::string & topic, const std::string & interface_type);
  DataStreamInstance * find(const std::string & topic);
  const DataStreamInstance * find(const std::string & topic) const;

  void setIntervalMs(const std::string & topic, int interval_ms);
  void start(const std::string & topic);
  void republish(const std::string & topic);
  void stop(const std::string & topic);

  bool onTrackPublished(const std::string & track_name, std::size_t generation);
  void onTrackFailed(const std::string & track_name);

  std::size_t generation() const;
  void resetSessionState();
  void shutdown();

private:
  using InstanceMap = std::unordered_map<std::string, std::shared_ptr<DataStreamInstance>>;
  using TrackMap = std::unordered_map<std::string, std::string>;

  std::shared_ptr<DataStreamInstance> requireInstance(const std::string & topic) const;
  std::shared_ptr<DataStreamInstance> findInstanceByTrackName(const std::string & track_name) const;
  void clearInstances(bool reopen_gate);

  rclcpp::Node & node_;
  RoomConnection & room_connection_;
  const SubscriptionQosConfig * qos_config_;
  QuiesceGate callback_gate_;
  std::atomic<bool> is_shutdown_{false};
  std::atomic<std::size_t> generation_{0};
  InstanceMap instances_;
  TrackMap topics_by_track_name_;
};

}  // namespace livekit_ros2_bridge
