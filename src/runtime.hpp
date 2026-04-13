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
class ControlPacketRouter;
class SubscriptionHeartbeatProcessor;
class RosExecutorQueue;
class RosServiceCaller;
class SubscriptionRegistry;
class RosTopicPublisher;
class VideoStreamRegistry;
class VideoProfilingRegistry;

struct FailFastCallbacks
{
  // Optional test callback to override process-wide ROS shutdown during fail-fast.
  std::function<void()> shutdown_callback;
  // Optional test callback to override process exit during fail-fast.
  std::function<void(int)> exit_callback;
};

// Wires one RoomConnection to the ROS-facing ingress helpers, publication owners,
// RPC handlers, and in-process video streams for a node.
// Construction performs eager startup; destruction shuts the room connection down before the ROS
// ingress helpers are torn down.
class Runtime final
{
public:
  Runtime(
    rclcpp::Node & node,
    std::unique_ptr<RoomConnection> connection,
    RuntimeConfig config,
    FailFastCallbacks fail_fast_callbacks = {});
  ~Runtime();

  // Idempotently begins teardown. RPC methods are unregistered before stop() so no new room
  // ingress reaches ROS while shutdown is in progress, and the executor queue is then shut down
  // after already-running work has had a chance to drain.
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  struct Components
  {
    std::unique_ptr<RoomConnection> room_connection;
    std::unique_ptr<RosExecutorQueue> ros_executor_queue;
    std::unique_ptr<RpcRouter> rpc_router;
    std::unique_ptr<RosTopicPublisher> ros_topic_publisher;
    std::unique_ptr<VideoStreamRegistry> video_stream_registry;
    std::unique_ptr<VideoProfilingRegistry> video_profiling_registry;
    std::unique_ptr<SubscriptionRegistry> subscription_registry;
    std::unique_ptr<SubscriptionHeartbeatProcessor> subscription_heartbeat_processor;
    std::unique_ptr<RosServiceCaller> ros_service_caller;
    std::unique_ptr<ControlPacketRouter> control_packet_router;
  };

  struct Config
  {
    VideoStreamConfig video_stream;
    SubscriptionQosConfig subscription_qos;
    std::string room;
    FailFastCallbacks fail_fast_callbacks;
    bool fail_fast_enabled = false;
    std::chrono::milliseconds fail_fast_disconnect_grace{0};
  };

  struct Timers
  {
    rclcpp::TimerBase::SharedPtr lease_gc;
    rclcpp::TimerBase::SharedPtr fail_fast;
    rclcpp::TimerBase::SharedPtr video_profile_summary;
  };

  struct State
  {
    struct FailFastTrigger
    {
      bool ready_once = false;
      std::string reason;
    };

    // Shared by RoomConnection-managed callback threads, ROS wall timers, and explicit shutdown.
    // These helpers keep the readiness/fail-fast state internally synchronized because those call
    // paths do not all run on the ROS executor.
    // Startup becomes ready only after both transport connectivity and required RPC registration
    // succeed. Either prerequisite may complete first; markRoomConnected() and
    // markRpcRegistered() return true only on the transition that satisfies the second
    // prerequisite for the first time.
    bool markRpcRegistered();
    bool markRoomConnected();
    void armGraceDeadline(std::chrono::milliseconds grace);
    void markDisconnected(const std::string & reason, bool fail_fast, std::chrono::milliseconds grace);
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
    EventThrottle executor_shutdown_enqueue_drop{std::chrono::seconds(5)};
    EventThrottle executor_unavailable_drop{std::chrono::seconds(5)};
    EventThrottle executor_shutdown_execute_drop{std::chrono::seconds(5)};
    mutable EventThrottle control_packet_shutdown_drop{std::chrono::seconds(5)};
    mutable EventThrottle control_packet_router_unavailable_drop{std::chrono::seconds(5)};
  };

  void checkFailFast();
  void logReady() const;
  void handleConnectionReset();
  void handleParticipantDisconnected(std::string requester_identity);
  void handleIncomingControlPacket(const IncomingControlPacket & packet);
  void logControlPacketDrop(const IncomingControlPacket & packet, const char * reason, EventThrottle & throttle) const;
  void logExecutorWorkDrop(const char * reason, const char * stage, EventThrottle & throttle);
  // Funnels RoomConnection ingress back onto the ROS executor queue so ROS-facing state changes
  // stay ordered with session reset and teardown. Work accepted before shutdown may still execute
  // if it reaches the queue before the executor is shut down.
  void submitToExecutor(std::function<void()> work);

  rclcpp::Node & node_;
  Components components_;
  Config config_;
  Timers timers_;
  State state_;
  Diagnostics diagnostics_;
};

}  // namespace livekit_ros2_bridge
