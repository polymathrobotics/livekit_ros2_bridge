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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "runtime_config.hpp"
#include "utils/event_throttle.hpp"

namespace livekit_ros2_bridge
{

class RpcRouter;
class CdrTrackPublisher;
class ControlPacketRouter;
class SubscriptionHeartbeatProcessor;
class RosExecutorQueue;
class RosServiceCaller;
class SubscriptionRegistry;
class RosTopicPublisher;
class VideoStreamManager;

// Wires one RoomSession to the ROS-facing publishers, RPC handlers, and in-process video streams
// for a node.
// Construction performs eager startup; destruction shuts the room session down before the ROS
// ingress pipeline is torn down.
class Runtime final
{
public:
  Runtime(rclcpp::Node & node, std::unique_ptr<RoomSession> session, RuntimeConfig runtime_config);
  ~Runtime();

  // Idempotently begins teardown. RPC methods are unregistered before stop() so no new room
  // ingress reaches ROS while shutdown is in progress, and the executor queue is then shut down
  // after already-running work has had a chance to drain.
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  bool isShuttingDown() const;
  // Drops new ingress once shutdown starts. Work accepted before shutdown may still execute if it
  // reaches the ROS executor before the queue is shut down.
  void submitExecutorWork(std::function<void()> fn);
  void handleIncomingControlPacket(const IncomingControlPacket & packet) const;

  rclcpp::Node & node_;
  std::unique_ptr<RoomSession> room_session_;
  std::unique_ptr<RosExecutorQueue> ros_executor_queue_;
  std::unique_ptr<RpcRouter> rpc_router_;
  std::unique_ptr<CdrTrackPublisher> cdr_track_publisher_;
  std::unique_ptr<RosTopicPublisher> ros_topic_publisher_;
  std::unique_ptr<VideoStreamManager> video_stream_manager_;
  std::unique_ptr<SubscriptionRegistry> subscription_registry_;
  std::unique_ptr<SubscriptionHeartbeatProcessor> subscription_heartbeat_processor_;
  std::unique_ptr<RosServiceCaller> ros_service_caller_;
  std::unique_ptr<ControlPacketRouter> control_packet_router_;
  VideoConfig video_config_;
  rclcpp::TimerBase::SharedPtr lease_gc_timer_;
  std::string room_;
  std::string identity_;
  std::atomic<bool> shutting_down_{false};
  EventThrottle executor_shutdown_enqueue_drop_throttle_{std::chrono::seconds(5)};
  EventThrottle executor_unavailable_drop_throttle_{std::chrono::seconds(5)};
  EventThrottle executor_shutdown_execute_drop_throttle_{std::chrono::seconds(5)};
  mutable EventThrottle control_packet_shutdown_drop_throttle_{std::chrono::seconds(5)};
  mutable EventThrottle control_packet_router_unavailable_drop_throttle_{std::chrono::seconds(5)};
};

}  // namespace livekit_ros2_bridge
