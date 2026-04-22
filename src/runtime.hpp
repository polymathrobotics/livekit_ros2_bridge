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
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "runtime_config.hpp"
#include "utils/event_throttle.hpp"
#include "video_profiling.hpp"

namespace livekit_ros2_bridge
{

class RpcRouter;
class PacketRouter;
class RosExecutorQueue;
class RosServiceCaller;
class DataStreamRegistry;
class SubscriptionLeaseManager;
class RosTopicPublisher;
class VideoStreamRegistry;
class VideoProfilingRegistry;

/// ## Startup order
///
/// Construction is eager rather than lazy:
///
/// 1. `Runtime` builds `RosExecutorQueue`, `DataTrackPublisher`, `RosTopicPublisher`, `SubscriptionRegistry`, `SubscriptionHeartbeatProcessor`, `RosServiceCaller`, `RpcRouter`, and `PacketRouter`.
/// 2. It creates a one-second lease GC timer. That timer also hops back through `submitExecutorWork()`.
/// 3. It starts `RoomConnection` with callbacks for connection reset, participant disconnect, and incoming data-packet-topic messages.
/// 4. After the connection thread is running, it registers the LiveKit RPC methods.
///
/// That order matters. The ROS-side helpers exist before the connection can emit callbacks, and the RPC surface is not exposed until the runtime has everything needed to serve those calls.
///
/// It runs one background loop:
///
/// - try to connect once
/// - wait for disconnect or stop
/// - clear per-connection room state
/// - reconnect with exponential backoff unless stop was requested
class Runtime final
{
public:
  Runtime(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config);
  ~Runtime();

  /// The key invariant is that the connection stops before the executor queue is torn down. Already
  /// accepted work may still be draining at that point, so the runtime also checks the shutdown
  /// flag inside queued lambdas.
  ///
  /// 1. flip the shutdown flag so new work is dropped
  /// 2. stop the lease GC timer
  /// 3. unregister RPC methods from the active connection
  /// 4. stop `RoomConnection` so no new SDK callbacks can enqueue ROS work
  /// 5. shut down `RosExecutorQueue`
  /// 6. shut down `SubscriptionRegistry`, unpublish data tracks, stop video streams, shut down `RosServiceCaller`, and clear cached ROS topic publishers
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  rclcpp::Node & node_;
  RuntimeConfig config_;

  std::atomic<bool> shutting_down_{false};
  std::mutex watchdog_mutex_;
  std::optional<SteadyClock::time_point> watchdog_deadline_;
  std::optional<SteadyClock::time_point> watchdog_unhealthy_since_;

  std::unique_ptr<RoomConnection> room_connection_;
  std::unique_ptr<RosExecutorQueue> ros_executor_queue_;
  std::unique_ptr<RpcRouter> rpc_router_;
  std::unique_ptr<RosTopicPublisher> ros_topic_publisher_;
  std::unique_ptr<DataStreamRegistry> data_stream_registry_;
  std::unique_ptr<VideoStreamRegistry> video_stream_registry_;
  std::unique_ptr<VideoProfilingRegistry> profiling_registry_;
  std::unique_ptr<SubscriptionLeaseManager> subscription_lease_manager_;
  std::unique_ptr<RosServiceCaller> ros_service_caller_;
  std::unique_ptr<PacketRouter> packet_router_;

  rclcpp::TimerBase::SharedPtr subscription_lease_gc_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr video_profile_summary_timer_;

  // Throttle repeated drop logs so disconnect and shutdown bursts stay readable.
  EventThrottle executor_shutdown_enqueue_drop_{std::chrono::seconds(5)};
  EventThrottle executor_unavailable_drop_{std::chrono::seconds(5)};
  EventThrottle executor_shutdown_execute_drop_{std::chrono::seconds(5)};

  void onRoomConnected();
  void onRoomIncomingPacket(const IncomingPacket & packet);
  void onRoomRemoteParticipantDisconnected(std::string remote_participant_identity);
  void onRoomReconnectRequested(const std::string & reason);
  void onRoomReconnecting(const std::string & reason);
  void onRoomReconnected();
  void onRoomConnectionReset();
  void setWatchdogHealthy(std::string_view reason);
  void setWatchdogUnhealthy(std::string_view reason);

  // Funnels RoomConnection ingress back onto the ROS executor queue so ROS-facing state changes
  // stay ordered with session reset and teardown. Work accepted before shutdown may still execute
  // if it reaches the queue before the executor is shut down.
  void submitToExecutor(std::function<void()> work);

  void checkWatchdog();
};

}  // namespace livekit_ros2_bridge
