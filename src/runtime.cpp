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

#include "runtime.hpp"

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

#include "control_packet_router.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "rpc_router.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "subscription_registry.hpp"
#include "topic_publish_command.hpp"
#include "topic_publisher.hpp"
#include "utils/log_event.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr auto kReconnectInitialBackoff = std::chrono::milliseconds(500);
constexpr auto kReconnectMaxBackoff = std::chrono::milliseconds(10000);
constexpr auto kFailFastEvaluationInterval = std::chrono::milliseconds(250);
constexpr auto kFailFastExitDelay = std::chrono::milliseconds(100);
}  // namespace

Runtime::Runtime(
  rclcpp::Node & node,
  std::unique_ptr<RoomSession> session,
  RuntimeConfig runtime_config,
  FailFastCallbacks fail_fast_callbacks)
: node_(node)
, room_session_(std::move(session))
, video_stream_config_(std::move(runtime_config.video_stream_config))
, subscription_qos_config_(std::move(runtime_config.subscription_qos_config))
, room_(runtime_config.room_connection_config.room)
, fail_fast_callbacks_(std::move(fail_fast_callbacks))
, fail_fast_enabled_(runtime_config.health_config.fail_fast_enabled)
, fail_fast_disconnect_grace_(runtime_config.health_config.fail_fast_disconnect_grace)
{
  if (room_session_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit session");
  }
  if (!fail_fast_callbacks_.shutdown_callback) {
    fail_fast_callbacks_.shutdown_callback = []() {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    };
  }
  if (!fail_fast_callbacks_.exit_callback) {
    fail_fast_callbacks_.exit_callback = [](int exit_code) { std::_Exit(exit_code); };
  }

  LogEvent(node_.get_logger(), "runtime_startup_begin")
    .field("phase", "startup")
    .fieldOr("room", room_, "<unset>")
    .info();

  ros_executor_queue_ = std::make_unique<RosExecutorQueue>(node_);
  ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(node_, runtime_config.access_policy);
  video_stream_registry_ = std::make_unique<VideoStreamRegistry>(node_, *room_session_, &subscription_qos_config_);

  subscription_registry_ = std::make_unique<SubscriptionRegistry>(
    node_, *room_session_, video_stream_registry_.get(), &video_stream_config_, &subscription_qos_config_);

  subscription_heartbeat_processor_ = std::make_unique<SubscriptionHeartbeatProcessor>(
    *subscription_registry_, *room_session_, runtime_config.access_policy, node_.get_clock());
  ros_service_caller_ = std::make_unique<RosServiceCaller>(node_);
  rpc_router_ =
    std::make_unique<RpcRouter>(node_, runtime_config.access_policy, *ros_executor_queue_, *ros_service_caller_);
  control_packet_router_ = std::make_unique<ControlPacketRouter>(
    node_.get_logger(),
    node_.get_clock(),
    ControlPacketRouter::Callbacks{
      [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
        submitExecutorWork(
          [this, requester_identity = std::move(requester_identity), heartbeat = std::move(heartbeat)]() {
            subscription_heartbeat_processor_->process(requester_identity, heartbeat);
          });
      },
      [this](std::string requester_identity, TopicPublishCommand command) {
        submitExecutorWork([this, requester_identity = std::move(requester_identity), command = std::move(command)]() {
          ros_topic_publisher_->publish(requester_identity, command);
        });
      },
    });

  lease_gc_timer_ = node_.create_wall_timer(kLeaseGcInterval, [this]() {
    submitExecutorWork([this]() {
      subscription_heartbeat_processor_->pruneExpiredSessionLeases();
      subscription_registry_->pruneExpiredLeases();
    });
  });
  if (fail_fast_enabled_) {
    std::lock_guard<std::mutex> lock(connection_state_mutex_);
    disconnect_deadline_ = SteadyClock::now() + fail_fast_disconnect_grace_;
  }
  fail_fast_timer_ = node_.create_wall_timer(kFailFastEvaluationInterval, [this]() { evaluateFailFast(); });

  room_session_->start(
    runtime_config.room_connection_config,
    runtime_config.access_token,
    RoomSessionCallbacks{
      [this]() { handleRoomConnected(); },
      [this](const std::string & reason) { handleReconnectRequested(reason); },
      [this]() {
        submitExecutorWork([this]() {
          subscription_registry_->resetSessionState();
          ros_service_caller_->resetSessionState();
        });
      },
      [this](const std::string & requester_identity) {
        const std::size_t gen = subscription_registry_->registryGeneration();
        submitExecutorWork([this, requester_identity, gen]() {
          // Keep leases alive across a browser refresh, but remember that the requester will
          // need fresh data-track publications because LiveKit binds those publications to the
          // old participant session that just disconnected.
          subscription_registry_->markRequesterForDataTrackRepublish(requester_identity, gen);
          ros_service_caller_->cancelCallsForRequester(requester_identity);
        });
      },
      [this](const IncomingControlPacket & packet) { handleIncomingControlPacket(packet); },
    },
    kReconnectInitialBackoff,
    kReconnectMaxBackoff);
  if (!rpc_router_->registerRpcMethods(*room_session_)) {
    LogEvent(node_.get_logger(), "runtime_startup_failed")
      .field("phase", "startup")
      .field("reason", "required_rpc_registration_failed")
      .fieldOr("room", room_, "<unset>")
      .error();
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }
  bool emit_ready_logs = false;
  {
    std::lock_guard<std::mutex> lock(connection_state_mutex_);
    rpc_methods_ready_ = true;
    if (connected_ && !ready_once_) {
      ready_once_ = true;
      emit_ready_logs = true;
    }
  }
  if (emit_ready_logs) {
    emitReadyLogs();
  }
}

Runtime::~Runtime()
{
  shutdown();
}

void Runtime::shutdown()
{
  if (shutting_down_.exchange(true)) {
    return;
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_start")
    .field("phase", "shutdown")
    .fieldOr("room", room_, "<unset>")
    .info();

  lease_gc_timer_.reset();
  fail_fast_timer_.reset();

  if (rpc_router_ != nullptr && room_session_ != nullptr) {
    rpc_router_->unregisterRpcMethods(*room_session_);
  }
  // Stop the room session before shutting down the executor queue so SDK callbacks can no longer
  // enqueue fresh ROS work while already-running executor tasks finish.
  if (room_session_ != nullptr) {
    room_session_->stop();
  }
  if (ros_executor_queue_ != nullptr) {
    ros_executor_queue_->shutdown();
  }
  if (subscription_registry_ != nullptr) {
    subscription_registry_->shutdown();
  }
  if (video_stream_registry_ != nullptr) {
    video_stream_registry_->shutdown();
  }
  if (ros_service_caller_ != nullptr) {
    ros_service_caller_->shutdown();
  }
  if (ros_topic_publisher_ != nullptr) {
    ros_topic_publisher_->shutdown();
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_complete")
    .field("phase", "shutdown")
    .fieldOr("room", room_, "<unset>")
    .info();
}

bool Runtime::isShuttingDown() const
{
  return shutting_down_.load();
}

void Runtime::handleRoomConnected()
{
  if (isShuttingDown()) {
    return;
  }

  bool emit_ready_logs = false;
  {
    std::lock_guard<std::mutex> lock(connection_state_mutex_);
    connected_ = true;
    disconnect_deadline_.reset();
    last_reconnect_reason_.clear();
    if (rpc_methods_ready_ && !ready_once_) {
      ready_once_ = true;
      emit_ready_logs = true;
    }
  }

  if (emit_ready_logs) {
    emitReadyLogs();
  }
}

void Runtime::handleReconnectRequested(const std::string & reason)
{
  if (isShuttingDown()) {
    return;
  }

  std::lock_guard<std::mutex> lock(connection_state_mutex_);
  connected_ = false;
  last_reconnect_reason_ = reason;
  if (fail_fast_enabled_) {
    disconnect_deadline_ = SteadyClock::now() + fail_fast_disconnect_grace_;
  } else {
    disconnect_deadline_.reset();
  }
}

void Runtime::evaluateFailFast()
{
  if (!fail_fast_enabled_ || isShuttingDown()) {
    return;
  }

  std::string disconnect_reason;
  bool ready_once = false;
  {
    std::lock_guard<std::mutex> lock(connection_state_mutex_);
    if (fail_fast_triggered_ || connected_ || !disconnect_deadline_.has_value()) {
      return;
    }
    if (SteadyClock::now() < *disconnect_deadline_) {
      return;
    }

    fail_fast_triggered_ = true;
    ready_once = ready_once_;
    if (ready_once_) {
      disconnect_reason = last_reconnect_reason_.empty() ? "reconnect_timeout" : last_reconnect_reason_;
    } else {
      disconnect_reason = "initial_connect_timeout";
    }
  }

  terminateForFailFast(disconnect_reason, ready_once);
}

void Runtime::emitReadyLogs()
{
  LogEvent(node_.get_logger(), "runtime_ready").field("phase", "startup").fieldOr("room", room_, "<unset>").info();
  LogEvent(node_.get_logger(), "node_ready").field("phase", "startup").fieldOr("room", room_, "<unset>").info();
}

void Runtime::terminateForFailFast(const std::string & disconnect_reason, bool ready_once)
{
  LogEvent(node_.get_logger(), "runtime_fail_fast_triggered")
    .field("phase", ready_once ? "reconnect" : "startup")
    .field("reason", "disconnect_grace_expired")
    .field("disconnect_reason", disconnect_reason)
    .fieldOr("room", room_, "<unset>")
    .field("grace_seconds", fail_fast_disconnect_grace_.count() / 1000.0)
    .field("ready_once", ready_once)
    .error();
  fail_fast_callbacks_.shutdown_callback();
  std::this_thread::sleep_for(kFailFastExitDelay);
  fail_fast_callbacks_.exit_callback(EXIT_FAILURE);
}

void Runtime::submitExecutorWork(std::function<void()> fn)
{
  auto logExecutorDrop = [this](const char * reason, const char * stage, EventThrottle & throttle) {
    if (const std::size_t count = throttle.recordAndTakePendingCount(); count > 0U) {
      LogEvent(node_.get_logger(), "executor_work_dropped")
        .field("reason", reason)
        .field("stage", stage)
        .field("count", count)
        .warn();
    }
  };

  if (isShuttingDown()) {
    logExecutorDrop("shutdown", "enqueue", executor_shutdown_enqueue_drop_throttle_);
    return;
  }
  if (ros_executor_queue_ == nullptr) {
    logExecutorDrop("executor_unavailable", "enqueue", executor_unavailable_drop_throttle_);
    return;
  }
  (void)ros_executor_queue_->submit([this, fn = std::move(fn)]() mutable {
    auto logExecutorDrop = [this](const char * reason, const char * stage, EventThrottle & throttle) {
      if (const std::size_t count = throttle.recordAndTakePendingCount(); count > 0U) {
        LogEvent(node_.get_logger(), "executor_work_dropped")
          .field("reason", reason)
          .field("stage", stage)
          .field("count", count)
          .warn();
      }
    };

    // The queue can still be draining work that was accepted before shutdown flipped the flag.
    if (isShuttingDown()) {
      logExecutorDrop("shutdown", "execute", executor_shutdown_execute_drop_throttle_);
      return;
    }
    fn();
  });
}

void Runtime::handleIncomingControlPacket(const IncomingControlPacket & packet) const
{
  auto logControlPacketDrop = [this, &packet](const char * reason, EventThrottle & throttle) {
    if (const std::size_t count = throttle.recordAndTakePendingCount(); count > 0U) {
      LogEvent(node_.get_logger(), "control_packet_dropped")
        .field("reason", reason)
        .field("control_topic", packet.control_topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .field("count", count)
        .warn();
    }
  };

  if (isShuttingDown()) {
    logControlPacketDrop("shutdown", control_packet_shutdown_drop_throttle_);
    return;
  }
  if (control_packet_router_ == nullptr) {
    logControlPacketDrop("router_unavailable", control_packet_router_unavailable_drop_throttle_);
    return;
  }
  control_packet_router_->route(packet);
}

}  // namespace livekit_ros2_bridge
