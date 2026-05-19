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
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "access_policy.hpp"
#include "protocol/subscriptions.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/timer.hpp"
#include "utils/event_throttle.hpp"
#include "video/stream_spec.hpp"

namespace livekit_ros2_bridge
{

class DataTrackPublisher;
class RoomConnection;
struct SubscriptionQosConfig;

namespace video
{
class TrackPublisher;
}  // namespace video

// Maintains per-requester leases for shared publishers and reports subscription status.
class SubscriptionLeaseManager final
{
  struct Subscription;

public:
  using Clock = std::chrono::steady_clock;
  using ExecutorSubmitter = std::function<void(std::function<void()> work)>;

  SubscriptionLeaseManager(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::Clock::SharedPtr clock,
    RoomConnection & room_connection,
    AccessPolicy access_policy,
    const SubscriptionQosConfig * qos_config = nullptr,
    const video::StreamConfig * video_stream_config = nullptr,
    Clock::duration heartbeat_lease_duration = std::chrono::seconds(45));
  ~SubscriptionLeaseManager();

  // Runs lease expiry through the executor path shared with heartbeat handling.
  void startPruneTimer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers,
    ExecutorSubmitter submit_to_executor);
  void handleHeartbeatPayload(const std::string & requester_identity, const std::vector<std::uint8_t> & payload);
  void pruneExpiredLeases();
  void shutdown();

private:
  struct SessionLease
  {
    std::string requester_identity;
    Clock::time_point expiry;
  };

  struct Lease
  {
    int preferred_interval_ms = 0;
    Clock::time_point expiry;
  };

  using DataPublisher = std::shared_ptr<DataTrackPublisher>;
  using VideoPublisher = std::shared_ptr<video::TrackPublisher>;
  using Runtime = std::variant<DataPublisher, VideoPublisher>;

  struct Subscription
  {
    SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string interface_type;
    std::map<std::string, Lease> leases;
    Runtime runtime;
  };

  struct ResolvedDemand
  {
    SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
    std::string name;
    std::string key;
    std::string interface_type;
    int preferred_interval_ms = 0;
    std::optional<video::StreamSpec> video_spec;
  };

  using Subscriptions = std::unordered_map<std::string, Subscription>;

  static constexpr auto kLogThrottle = std::chrono::seconds(5);
  static int minimumIntervalMs(const std::map<std::string, Lease> & leases);

  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeGraphInterface>
    node_interfaces_;
  rclcpp::Clock::SharedPtr clock_;
  RoomConnection & room_connection_;
  AccessPolicy access_policy_;
  const SubscriptionQosConfig * qos_config_;
  const video::StreamConfig * video_stream_config_;
  Clock::duration heartbeat_lease_duration_;

  std::atomic<bool> is_shutdown_{false};
  rclcpp::TimerBase::SharedPtr prune_timer_;

  // Browser-tab scoped leases resolve anonymous heartbeats when LiveKit omits identity.
  std::unordered_map<std::string, SessionLease> session_leases_;

  // Canonical target (`kind:name`) to the data or video runtime shared by live requesters.
  Subscriptions subscriptions_;

  EventThrottle conflict_throttle_{kLogThrottle};

  void handleHeartbeat(const std::string & requester_identity, const SubscriptionHeartbeat & heartbeat);
  std::optional<std::string> resolveIdentity(
    const std::string & requester_identity, const std::optional<std::string> & session_id);
  void renewSessionLease(
    const std::string & requester_identity, const std::optional<std::string> & session_id, Clock::time_point expiry);
  void pruneSessionLeases(Clock::time_point now);
  const video::StreamConfig & videoStreamConfig() const;
  video::StreamSpec resolveVideoSpec(
    SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const;
  ResolvedDemand resolveDemand(const SubscriptionDemand & demand) const;
  void resolveDemandDelivery(ResolvedDemand & demand) const;
  SubscriptionStatus create(const ResolvedDemand & demand, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus renew(Subscription & subscription, const std::string & requester_identity, const Lease & lease);
  SubscriptionStatus ensure(
    const std::string & requester_identity, const ResolvedDemand & demand, Clock::time_point expiry);
  SubscriptionStatus status(const Subscription & subscription) const;
  SubscriptionStatusReport createStatusReport(
    const SubscriptionHeartbeat & heartbeat, const std::string & requester_identity, Clock::time_point expiry);
  void appendDemandStatus(
    SubscriptionStatusReport & report,
    const std::string & requester_identity,
    const SubscriptionDemand & demand,
    Clock::time_point expiry);
  void publishStatusReport(
    const std::string & requester_identity,
    const std::optional<std::string> & session_id,
    const SubscriptionStatusReport & report);

  void pruneLeases(Clock::time_point now);
};

}  // namespace livekit_ros2_bridge
