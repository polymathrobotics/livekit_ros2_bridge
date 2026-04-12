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
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "subscription_registry.hpp"
#include "topic_publish_command.hpp"
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
  std::unique_ptr<RoomConnection> room_connection,
  RuntimeConfig runtime_config,
  FailFastCallbacks fail_fast_callbacks)
: node_(node)
, config_{
    std::move(runtime_config.video_stream_config),
    std::move(runtime_config.subscription_qos_config),
    runtime_config.room_connection_config.room,
    std::move(fail_fast_callbacks),
    runtime_config.health_config.fail_fast_enabled,
    runtime_config.health_config.fail_fast_disconnect_grace,
  }
{
  components_.room_connection = std::move(room_connection);
  if (components_.room_connection == nullptr) {
    throw std::runtime_error("Failed to create LiveKit room connection");
  }
  if (!config_.fail_fast_callbacks.shutdown_callback) {
    config_.fail_fast_callbacks.shutdown_callback = []() {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    };
  }
  if (!config_.fail_fast_callbacks.exit_callback) {
    config_.fail_fast_callbacks.exit_callback = [](int exit_code) { std::_Exit(exit_code); };
  }

  LogEvent(node_.get_logger(), "runtime_startup_begin")
    .field("phase", "startup")
    .fieldOr("room", config_.room, "<unset>")
    .info();

  if (runtime_config.video_profiling_config.enabled) {
    components_.video_profiling_registry =
      std::make_unique<VideoProfilingRegistry>(node_.get_logger(), std::move(runtime_config.video_profiling_config));
    components_.video_profiling_registry->emitEnabledLog();
  }

  components_.ros_executor_queue = std::make_unique<RosExecutorQueue>(node_);
  components_.ros_topic_publisher = std::make_unique<RosTopicPublisher>(node_, runtime_config.access_policy);
  components_.video_stream_registry = std::make_unique<VideoStreamRegistry>(
    node_, *components_.room_connection, &config_.subscription_qos, components_.video_profiling_registry.get());

  components_.subscription_registry = std::make_unique<SubscriptionRegistry>(
    node_,
    *components_.room_connection,
    components_.video_stream_registry.get(),
    &config_.video_stream,
    &config_.subscription_qos);

  components_.subscription_heartbeat_processor = std::make_unique<SubscriptionHeartbeatProcessor>(
    *components_.subscription_registry, *components_.room_connection, runtime_config.access_policy, node_.get_clock());

  components_.ros_service_caller = std::make_unique<RosServiceCaller>(node_);
  components_.rpc_router = std::make_unique<RpcRouter>(
    node_, runtime_config.access_policy, *components_.ros_executor_queue, *components_.ros_service_caller);
  components_.control_packet_router = std::make_unique<ControlPacketRouter>(
    node_.get_logger(),
    node_.get_clock(),
    ControlPacketRouter::Callbacks{
      [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
        submitExecutorWork(
          [this, requester_identity = std::move(requester_identity), heartbeat = std::move(heartbeat)]() {
            components_.subscription_heartbeat_processor->process(requester_identity, heartbeat);
          });
      },
      [this](std::string requester_identity, TopicPublishCommand command) {
        submitExecutorWork([this, requester_identity = std::move(requester_identity), command = std::move(command)]() {
          components_.ros_topic_publisher->publish(requester_identity, command);
        });
      },
    });

  timers_.lease_gc = node_.create_wall_timer(kLeaseGcInterval, [this]() {
    submitExecutorWork([this]() {
      components_.subscription_heartbeat_processor->pruneExpiredClientSessionLeases();
      components_.subscription_registry->pruneExpiredLeases();
    });
  });
  if (config_.fail_fast_enabled) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.disconnect_deadline = SteadyClock::now() + config_.fail_fast_disconnect_grace;
  }
  timers_.fail_fast = node_.create_wall_timer(kFailFastEvaluationInterval, [this]() { evaluateFailFast(); });
  if (components_.video_profiling_registry != nullptr && components_.video_profiling_registry->enabled()) {
    timers_.video_profile_summary = node_.create_wall_timer(
      components_.video_profiling_registry->config().summary_interval,
      [this]() { components_.video_profiling_registry->emitSummaryLogs(); });
  }

  components_.room_connection->start(
    runtime_config.room_connection_config,
    runtime_config.access_token,
    RoomConnectionCallbacks{
      [this]() { handleRoomConnected(); },
      [this](const std::string & reason) { handleReconnectRequested(reason); },
      [this]() {
        submitExecutorWork([this]() {
          components_.subscription_registry->resetSessionState();
          components_.ros_service_caller->resetSessionState();
        });
      },
      [this](const std::string & requester_identity) {
        const std::size_t gen = components_.subscription_registry->registryGeneration();
        submitExecutorWork([this, requester_identity, gen]() {
          // Keep leases alive across a browser refresh, but remember that the requester will
          // need fresh data-track publications because LiveKit binds those publications to the
          // old participant_session that just disconnected.
          components_.subscription_registry->markRequesterForDataTrackRepublish(requester_identity, gen);
          components_.ros_service_caller->cancelCallsForRequester(requester_identity);
        });
      },
      [this](const IncomingControlPacket & packet) { handleIncomingControlPacket(packet); },
    },
    kReconnectInitialBackoff,
    kReconnectMaxBackoff);

  if (!components_.rpc_router->registerRpcMethods(*components_.room_connection)) {
    LogEvent(node_.get_logger(), "runtime_startup_failed")
      .field("phase", "startup")
      .field("reason", "required_rpc_registration_failed")
      .fieldOr("room", config_.room, "<unset>")
      .error();
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }
  bool emit_ready_logs = false;
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.rpc_methods_ready = true;
    if (state_.connected && !state_.ready_once) {
      state_.ready_once = true;
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
  if (state_.shutting_down.exchange(true)) {
    return;
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_start")
    .field("phase", "shutdown")
    .fieldOr("room", config_.room, "<unset>")
    .info();

  timers_.lease_gc.reset();
  timers_.fail_fast.reset();
  timers_.video_profile_summary.reset();

  if (components_.rpc_router != nullptr && components_.room_connection != nullptr) {
    components_.rpc_router->unregisterRpcMethods(*components_.room_connection);
  }
  // Stop the room connection before shutting down the executor queue so SDK callbacks can no longer
  // enqueue fresh ROS work while already-running executor tasks finish.
  if (components_.room_connection != nullptr) {
    components_.room_connection->stop();
  }
  if (components_.ros_executor_queue != nullptr) {
    components_.ros_executor_queue->shutdown();
  }
  if (components_.subscription_registry != nullptr) {
    components_.subscription_registry->shutdown();
  }
  if (components_.video_stream_registry != nullptr) {
    components_.video_stream_registry->shutdown();
  }
  if (components_.ros_service_caller != nullptr) {
    components_.ros_service_caller->shutdown();
  }
  if (components_.ros_topic_publisher != nullptr) {
    components_.ros_topic_publisher->shutdown();
  }
  if (components_.video_profiling_registry != nullptr) {
    components_.video_profiling_registry->emitSummaryLogs();
    components_.video_profiling_registry->flushTrace();
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_complete")
    .field("phase", "shutdown")
    .fieldOr("room", config_.room, "<unset>")
    .info();
}

bool Runtime::isShuttingDown() const
{
  return state_.shutting_down.load();
}

void Runtime::handleRoomConnected()
{
  if (isShuttingDown()) {
    return;
  }

  bool emit_ready_logs = false;
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.connected = true;
    state_.disconnect_deadline.reset();
    state_.last_reconnect_reason.clear();
    if (state_.rpc_methods_ready && !state_.ready_once) {
      state_.ready_once = true;
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

  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.connected = false;
  state_.last_reconnect_reason = reason;
  if (config_.fail_fast_enabled) {
    state_.disconnect_deadline = SteadyClock::now() + config_.fail_fast_disconnect_grace;
  } else {
    state_.disconnect_deadline.reset();
  }
}

void Runtime::evaluateFailFast()
{
  if (!config_.fail_fast_enabled || isShuttingDown()) {
    return;
  }

  std::string disconnect_reason;
  bool ready_once = false;
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    if (state_.fail_fast_triggered || state_.connected || !state_.disconnect_deadline.has_value()) {
      return;
    }
    if (SteadyClock::now() < *state_.disconnect_deadline) {
      return;
    }

    state_.fail_fast_triggered = true;
    ready_once = state_.ready_once;
    if (state_.ready_once) {
      disconnect_reason = state_.last_reconnect_reason.empty() ? "reconnect_timeout" : state_.last_reconnect_reason;
    } else {
      disconnect_reason = "initial_connect_timeout";
    }
  }

  terminateForFailFast(disconnect_reason, ready_once);
}

void Runtime::emitReadyLogs()
{
  LogEvent(node_.get_logger(), "runtime_ready")
    .field("phase", "startup")
    .fieldOr("room", config_.room, "<unset>")
    .info();
  LogEvent(node_.get_logger(), "node_ready").field("phase", "startup").fieldOr("room", config_.room, "<unset>").info();
}

void Runtime::terminateForFailFast(const std::string & disconnect_reason, bool ready_once)
{
  LogEvent(node_.get_logger(), "runtime_fail_fast_triggered")
    .field("phase", ready_once ? "reconnect" : "startup")
    .field("reason", "disconnect_grace_expired")
    .field("disconnect_reason", disconnect_reason)
    .fieldOr("room", config_.room, "<unset>")
    .field("grace_seconds", config_.fail_fast_disconnect_grace.count() / 1000.0)
    .field("ready_once", ready_once)
    .error();
  config_.fail_fast_callbacks.shutdown_callback();
  std::this_thread::sleep_for(kFailFastExitDelay);
  config_.fail_fast_callbacks.exit_callback(EXIT_FAILURE);
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
    logExecutorDrop("shutdown", "enqueue", diagnostics_.executor_shutdown_enqueue_drop);
    return;
  }
  if (components_.ros_executor_queue == nullptr) {
    logExecutorDrop("executor_unavailable", "enqueue", diagnostics_.executor_unavailable_drop);
    return;
  }
  (void)components_.ros_executor_queue->submit([this, fn = std::move(fn)]() mutable {
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
      logExecutorDrop("shutdown", "execute", diagnostics_.executor_shutdown_execute_drop);
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
    logControlPacketDrop("shutdown", diagnostics_.control_packet_shutdown_drop);
    return;
  }
  if (components_.control_packet_router == nullptr) {
    logControlPacketDrop("router_unavailable", diagnostics_.control_packet_router_unavailable_drop);
    return;
  }
  components_.control_packet_router->route(packet);
}

}  // namespace livekit_ros2_bridge
