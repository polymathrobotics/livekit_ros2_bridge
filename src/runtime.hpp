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
    std::unique_ptr<RoomConnection> room_connection,
    RuntimeConfig runtime_config,
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

  struct ConnectionState
  {
    std::atomic<bool> shutting_down{false};
    mutable std::mutex mutex;

    // Guarded by mutex.
    bool connected = false;
    bool ready_once = false;
    bool rpc_methods_ready = false;
    bool fail_fast_triggered = false;
    std::optional<SteadyClock::time_point> disconnect_deadline;
    std::string last_reconnect_reason;
  };

  struct Diagnostics
  {
    EventThrottle executor_shutdown_enqueue_drop{std::chrono::seconds(5)};
    EventThrottle executor_unavailable_drop{std::chrono::seconds(5)};
    EventThrottle executor_shutdown_execute_drop{std::chrono::seconds(5)};
    mutable EventThrottle control_packet_shutdown_drop{std::chrono::seconds(5)};
    mutable EventThrottle control_packet_router_unavailable_drop{std::chrono::seconds(5)};
  };

  bool isShuttingDown() const;
  void handleRoomConnected();
  void handleReconnectRequested(const std::string & reason);
  void evaluateFailFast();
  void emitReadyLogs();
  void terminateForFailFast(const std::string & disconnect_reason, bool ready_once);
  // Drops new ingress once shutdown starts. Work accepted before shutdown may still execute if it
  // reaches the ROS executor before the queue is shut down.
  void submitExecutorWork(std::function<void()> fn);
  void handleIncomingControlPacket(const IncomingControlPacket & packet) const;

  rclcpp::Node & node_;
  Components components_;
  Config config_;
  Timers timers_;
  ConnectionState state_;
  Diagnostics diagnostics_;
};

}  // namespace livekit_ros2_bridge
