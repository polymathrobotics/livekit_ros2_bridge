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
  std::atomic<bool> shutting_down_{false};
  std::mutex watchdog_mutex_;
  std::optional<SteadyClock::time_point> watchdog_deadline_;

  void checkWatchdog();
  void onConnectionReset();
  void onIncomingPacket(const IncomingPacket & packet);
  void onParticipantDisconnected(std::string requester_identity);
  void onReconnectRequested(const std::string & reason);
  void onConnected();

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
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr video_profile_summary_timer_;

  // Throttle repeated drop logs so disconnect and shutdown bursts stay readable.
  EventThrottle executor_shutdown_enqueue_drop_{std::chrono::seconds(5)};
  EventThrottle executor_unavailable_drop_{std::chrono::seconds(5)};
  EventThrottle executor_shutdown_execute_drop_{std::chrono::seconds(5)};
};

}  // namespace livekit_ros2_bridge
