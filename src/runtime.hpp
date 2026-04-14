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

#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "runtime_config.hpp"
#include "utils/event_throttle.hpp"
#include "video_profiling.hpp"

namespace livekit_ros2_bridge
{

class RpcRouter;
class PacketRouter;
class SubscriptionHeartbeatProcessor;
class RosExecutorQueue;
class RosServiceCaller;
class SubscriptionRegistry;
class RosTopicPublisher;
class VideoStreamRegistry;
class VideoProfilingRegistry;

// Wires one RoomConnection to the ROS-facing ingress helpers, publication owners,
// RPC handlers, and in-process video streams for a node.
// Construction performs eager startup; destruction shuts the room connection down before the ROS
// ingress helpers are torn down.
class Runtime final
{
public:
  Runtime(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config);
  ~Runtime();

  // Idempotently begins teardown. RPC methods are unregistered before stop() so no new room
  // ingress reaches ROS while shutdown is in progress, and the executor queue is then shut down
  // after already-running work has had a chance to drain.
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  struct State
  {
    struct ConnectTransition
    {
      bool became_ready = false;
      bool recovered = false;
      std::string disconnect_reason;
    };

    struct DisconnectTransition
    {
      bool ready_once = false;
    };

    struct FailFastTrigger
    {
      bool ready_once = false;
      std::string reason;
    };

    // Shared by RoomConnection-managed callback threads, ROS wall timers, and explicit shutdown.
    // These helpers keep the readiness/fail-fast state internally synchronized because those call
    // paths do not all run on the ROS executor.
    // Startup becomes ready only after both transport connectivity and required RPC registration
    // succeed. Either prerequisite may complete first; markRpcRegistered() returns true only on
    // the transition that satisfies the second prerequisite for the first time, while
    // markConnected() reports whether the connect completed initial readiness or recovered a
    // previously ready runtime.
    bool markRpcRegistered();
    ConnectTransition markConnected();
    void armGraceDeadline(std::chrono::milliseconds grace);
    DisconnectTransition markDisconnected(const std::string & reason, bool fail_fast, std::chrono::milliseconds grace);
    std::optional<FailFastTrigger> takeFailFastTrigger(SteadyClock::time_point now);

  private:
    void armGraceDeadlineLocked(std::chrono::milliseconds grace);
    bool markReadyLocked();

  public:
    std::atomic<bool> shutting_down{false};
    mutable std::mutex mutex;

    // Guarded by mutex.
    bool room_connected = false;
    // Sticky once the runtime has observed a fully usable session; reconnects clear
    // `room_connected` but leave this latched so fail-fast can distinguish startup from recovery.
    bool ready_once = false;
    // Tracks the local side of readiness because the room may connect before required RPC methods
    // finish registering.
    bool rpc_registered = false;
    // One-shot latch so fail-fast shutdown and exit only happen once per runtime instance.
    bool fail_fast_fired = false;
    // Armed during startup or a reconnect grace window and cleared once connectivity is healthy
    // again, or when fail-fast is disabled for the current reconnect attempt.
    std::optional<SteadyClock::time_point> grace_deadline;
    // Carries the latest disconnect cause into fail-fast logs; initial connect failures synthesize
    // their own reason instead.
    std::string reason;
  };

  struct Diagnostics
  {
    static constexpr auto kLogThrottle = std::chrono::seconds(5);

    EventThrottle executor_shutdown_enqueue_drop{kLogThrottle};
    EventThrottle executor_unavailable_drop{kLogThrottle};
    EventThrottle executor_shutdown_execute_drop{kLogThrottle};
    mutable EventThrottle packet_shutdown_drop{kLogThrottle};
    mutable EventThrottle packet_router_unavailable_drop{kLogThrottle};
  };

  void checkFailFast();
  void onConnectionReset();
  void onIncomingPacket(const IncomingPacket & packet);
  void onParticipantDisconnected(std::string requester_identity);
  void onReconnectRequested(const std::string & reason);
  void onConnected();
  void initPacketRouting();
  void initFailFast();
  void initRosInterfaces();
  void initSubscriptionRuntime();
  void initVideoProfiling();
  void logPacketDrop(const IncomingPacket & packet, const char * reason, EventThrottle & throttle) const;
  void logExecutorWorkDrop(const char * reason, const char * stage, EventThrottle & throttle);
  void startRoomConnection();

  // Funnels RoomConnection ingress back onto the ROS executor queue so ROS-facing state changes
  // stay ordered with session reset and teardown. Work accepted before shutdown may still execute
  // if it reaches the queue before the executor is shut down.
  void submitToExecutor(std::function<void()> work);

  rclcpp::Node & node_;
  RuntimeConfig config_;

  std::unique_ptr<RoomConnection> room_connection_;
  std::unique_ptr<RosExecutorQueue> ros_executor_queue_;
  std::unique_ptr<RpcRouter> rpc_router_;
  std::unique_ptr<RosTopicPublisher> ros_topic_publisher_;
  std::unique_ptr<VideoStreamRegistry> video_stream_registry_;
  std::unique_ptr<VideoProfilingRegistry> video_profiling_registry_;
  std::unique_ptr<SubscriptionRegistry> subscription_registry_;
  std::unique_ptr<SubscriptionHeartbeatProcessor> subscription_heartbeat_processor_;
  std::unique_ptr<RosServiceCaller> ros_service_caller_;
  std::unique_ptr<PacketRouter> packet_router_;

  rclcpp::TimerBase::SharedPtr subscription_lease_gc_timer_;
  rclcpp::TimerBase::SharedPtr fail_fast_timer_;
  rclcpp::TimerBase::SharedPtr video_profile_summary_timer_;
  State state_;
  Diagnostics diagnostics_;
};

}  // namespace livekit_ros2_bridge
