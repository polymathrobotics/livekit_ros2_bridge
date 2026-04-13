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

LogEvent runtimeLog(rclcpp::Logger logger, std::string_view event_name, const std::string & room)
{
  LogEvent event(std::move(logger), event_name);
  event.fieldOr("room", room, "<unset>");
  return event;
}
}  // namespace

bool Runtime::State::markRpcRegistered()
{
  std::lock_guard<std::mutex> lock(mutex);
  rpc_registered = true;
  return markReadyLocked();
}

Runtime::State::RoomConnectedTransition Runtime::State::markRoomConnected()
{
  std::lock_guard<std::mutex> lock(mutex);
  const bool was_connected = room_connected;
  const bool was_ready_once = ready_once;
  std::string disconnect_reason = std::move(reason);
  room_connected = true;
  grace_deadline.reset();
  reason.clear();

  RoomConnectedTransition transition;
  transition.became_ready = markReadyLocked();
  transition.recovered = was_ready_once && !was_connected;
  transition.disconnect_reason = std::move(disconnect_reason);
  return transition;
}

void Runtime::State::armGraceDeadline(std::chrono::milliseconds grace)
{
  std::lock_guard<std::mutex> lock(mutex);
  armGraceDeadlineLocked(grace);
}

Runtime::State::DisconnectTransition Runtime::State::markDisconnected(
  const std::string & reason, bool fail_fast, std::chrono::milliseconds grace)
{
  std::lock_guard<std::mutex> lock(mutex);
  DisconnectTransition transition;
  transition.ready_once = ready_once;
  // Reconnects preserve the startup latches so fail-fast can distinguish recovery from startup,
  // and required RPC methods stay registered across reconnect attempts.
  room_connected = false;
  this->reason = reason;
  if (fail_fast) {
    armGraceDeadlineLocked(grace);
  } else {
    grace_deadline.reset();
  }

  return transition;
}

std::optional<Runtime::State::FailFastTrigger> Runtime::State::takeFailFastTrigger(Runtime::SteadyClock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (fail_fast_fired || room_connected || !grace_deadline.has_value()) {
    return std::nullopt;
  }
  if (now < *grace_deadline) {
    return std::nullopt;
  }

  fail_fast_fired = true;
  if (!ready_once) {
    return FailFastTrigger{
      false,
      "initial_connect_timeout",
    };
  }

  return FailFastTrigger{
    true,
    this->reason.empty() ? std::string{"reconnect_timeout"} : this->reason,
  };
}

void Runtime::State::armGraceDeadlineLocked(std::chrono::milliseconds grace)
{
  grace_deadline = Runtime::SteadyClock::now() + grace;
}

bool Runtime::State::markReadyLocked()
{
  if (ready_once || !room_connected || !rpc_registered) {
    return false;
  }

  ready_once = true;
  return true;
}

Runtime::Runtime(
  rclcpp::Node & node,
  std::unique_ptr<RoomConnection> connection,
  RuntimeConfig config,
  FailFastCallbacks fail_fast_callbacks)
: node_(node)
, config_{
    std::move(config.video_stream_config),
    std::move(config.subscription_qos_config),
    config.room_connection_config.room,
    std::move(fail_fast_callbacks),
    config.health_config.fail_fast_enabled,
    config.health_config.fail_fast_disconnect_grace,
  }
{
  components_.room_connection = std::move(connection);
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

  runtimeLog(node_.get_logger(), "runtime_startup_begin", config_.room).info();

  if (config.video_profiling_config.enabled) {
    components_.video_profiling_registry =
      std::make_unique<VideoProfilingRegistry>(node_.get_logger(), std::move(config.video_profiling_config));
    components_.video_profiling_registry->logConfig();
  }

  components_.ros_executor_queue = std::make_unique<RosExecutorQueue>(node_);
  components_.ros_topic_publisher = std::make_unique<RosTopicPublisher>(node_, config.access_policy);
  components_.video_stream_registry = std::make_unique<VideoStreamRegistry>(
    node_, *components_.room_connection, &config_.subscription_qos, components_.video_profiling_registry.get());

  components_.subscription_registry = std::make_unique<SubscriptionRegistry>(
    node_,
    *components_.room_connection,
    components_.video_stream_registry.get(),
    &config_.video_stream,
    &config_.subscription_qos);

  components_.subscription_heartbeat_processor = std::make_unique<SubscriptionHeartbeatProcessor>(
    *components_.subscription_registry, *components_.room_connection, config.access_policy, node_.get_clock());

  components_.ros_service_caller = std::make_unique<RosServiceCaller>(node_);
  components_.rpc_router = std::make_unique<RpcRouter>(
    node_, config.access_policy, *components_.ros_executor_queue, *components_.ros_service_caller);
  // RoomConnection may invoke control-packet callbacks from its own worker threads. Parse and
  // dispatch immediately, but bounce ROS-visible side effects through submitToExecutor() so they stay
  // serialized with shutdown and other session-state mutations on the executor queue.
  ControlPacketRouter::Handlers handlers;
  handlers.heartbeat_handler = [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
    submitToExecutor([this, requester_identity = std::move(requester_identity), heartbeat = std::move(heartbeat)]() {
      components_.subscription_heartbeat_processor->process(requester_identity, heartbeat);
    });
  };
  handlers.publish_handler = [this](std::string requester_identity, TopicPublishCommand command) {
    submitToExecutor([this, requester_identity = std::move(requester_identity), command = std::move(command)]() {
      components_.ros_topic_publisher->publish(requester_identity, command);
    });
  };
  components_.control_packet_router =
    std::make_unique<ControlPacketRouter>(node_.get_logger(), node_.get_clock(), std::move(handlers));

  timers_.lease_gc = node_.create_wall_timer(kLeaseGcInterval, [this]() {
    submitToExecutor([this]() {
      components_.subscription_heartbeat_processor->pruneExpiredLeases();
      components_.subscription_registry->pruneExpiredLeases();
    });
  });
  if (config_.fail_fast_enabled) {
    state_.armGraceDeadline(config_.fail_fast_disconnect_grace);
  }
  timers_.fail_fast = node_.create_wall_timer(kFailFastEvaluationInterval, [this]() { checkFailFast(); });
  if (components_.video_profiling_registry != nullptr && components_.video_profiling_registry->enabled()) {
    timers_.video_profile_summary = node_.create_wall_timer(
      components_.video_profiling_registry->config().summary_interval,
      [this]() { components_.video_profiling_registry->logSummaries(); });
  }

  // `start()` may deliver callbacks before RPC registration completes, so readiness is logged when
  // the second prerequisite arrives.
  RoomConnectionCallbacks room_connection_callbacks{
    [this]() {
      if (state_.shutting_down.load()) {
        return;
      }

      const auto transition = state_.markRoomConnected();
      if (transition.became_ready) {
        logReady();
        return;
      }

      if (transition.recovered) {
        runtimeLog(node_.get_logger(), "runtime_reconnected", config_.room)
          .fieldOr("disconnect_reason", transition.disconnect_reason, "connection_lost")
          .info();
      }
    },
    [this](const std::string & reason) {
      if (state_.shutting_down.load()) {
        return;
      }

      const auto transition =
        state_.markDisconnected(reason, config_.fail_fast_enabled, config_.fail_fast_disconnect_grace);
      LogEvent disconnect_log = runtimeLog(node_.get_logger(), "runtime_disconnect_observed", config_.room);
      disconnect_log.field("phase", transition.ready_once ? "reconnect" : "startup")
        .fieldOr("disconnect_reason", reason, "connection_lost");

      if (!config_.fail_fast_enabled) {
        disconnect_log.info();
        return;
      }

      disconnect_log.field("grace_seconds", config_.fail_fast_disconnect_grace.count() / 1000.0);
      disconnect_log.warn();
    },
    [this]() { handleConnectionReset(); },
    [this](const std::string & requester_identity) { handleParticipantDisconnected(requester_identity); },
    [this](const IncomingControlPacket & packet) { handleIncomingControlPacket(packet); },
  };
  components_.room_connection->start(
    config.room_connection_config,
    config.access_token,
    std::move(room_connection_callbacks),
    kReconnectInitialBackoff,
    kReconnectMaxBackoff);

  if (!components_.rpc_router->registerRpcs(*components_.room_connection)) {
    runtimeLog(node_.get_logger(), "runtime_startup_failed", config_.room)
      .field("reason", "required_rpc_registration_failed")
      .error();
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }
  if (state_.markRpcRegistered()) {
    logReady();
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

  runtimeLog(node_.get_logger(), "runtime_shutdown_start", config_.room).info();

  // Disarm periodic work first so lease GC and fail-fast evaluation stop racing a deliberate
  // shutdown while the shared components below are being torn down.
  timers_.lease_gc.reset();
  timers_.fail_fast.reset();
  timers_.video_profile_summary.reset();

  if (components_.rpc_router != nullptr && components_.room_connection != nullptr) {
    components_.rpc_router->unregisterRpcs(*components_.room_connection);
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
    components_.video_profiling_registry->logSummaries();
    components_.video_profiling_registry->flushTrace();
  }

  runtimeLog(node_.get_logger(), "runtime_shutdown_complete", config_.room).info();
}

void Runtime::checkFailFast()
{
  if (!config_.fail_fast_enabled || state_.shutting_down.load()) {
    return;
  }

  const auto trigger = state_.takeFailFastTrigger(SteadyClock::now());
  if (!trigger.has_value()) {
    return;
  }

  runtimeLog(node_.get_logger(), "runtime_fail_fast_triggered", config_.room)
    .field("phase", trigger->ready_once ? "reconnect" : "startup")
    .field("disconnect_reason", trigger->reason)
    .field("grace_seconds", config_.fail_fast_disconnect_grace.count() / 1000.0)
    .error();
  // Give ROS shutdown and log flushing a brief head start before forcing process exit.
  config_.fail_fast_callbacks.shutdown_callback();
  std::this_thread::sleep_for(kFailFastExitDelay);
  config_.fail_fast_callbacks.exit_callback(EXIT_FAILURE);
}

void Runtime::logReady() const
{
  runtimeLog(node_.get_logger(), "runtime_ready", config_.room).info();
  runtimeLog(node_.get_logger(), "node_ready", config_.room).info();
}

void Runtime::handleConnectionReset()
{
  submitToExecutor([this]() {
    // Reset session-owned state on the ROS executor so cleanup stays ordered with any
    // in-flight heartbeat or RPC work targeting the old connection generation.
    components_.subscription_registry->resetSessionState();
    components_.ros_service_caller->resetSessionState();
  });
}

void Runtime::handleParticipantDisconnected(std::string requester_identity)
{
  const std::size_t generation = components_.subscription_registry->generation();
  submitToExecutor([this, requester_identity = std::move(requester_identity), generation]() {
    // Keep leases alive across a browser refresh, but queue fresh data-track publications
    // because LiveKit binds them to the old participant_session.
    components_.subscription_registry->queueDataTrackRepublish(requester_identity, generation);
    components_.ros_service_caller->cancelCallsForRequester(requester_identity);
  });
}

void Runtime::handleIncomingControlPacket(const IncomingControlPacket & packet)
{
  if (state_.shutting_down.load()) {
    logControlPacketDrop(packet, "shutdown", diagnostics_.control_packet_shutdown_drop);
    return;
  }
  if (components_.control_packet_router == nullptr) {
    logControlPacketDrop(packet, "router_unavailable", diagnostics_.control_packet_router_unavailable_drop);
    return;
  }
  components_.control_packet_router->route(packet);
}

void Runtime::logControlPacketDrop(
  const IncomingControlPacket & packet, const char * reason, EventThrottle & throttle) const
{
  const std::size_t count = throttle.recordAndTakePendingCount();
  if (count == 0U) {
    return;
  }

  LogEvent(node_.get_logger(), "control_packet_dropped")
    .field("reason", reason)
    .field("control_topic", packet.control_topic)
    .fieldOr("requester_identity", packet.requester_identity)
    .field("count", count)
    .warn();
}

void Runtime::logExecutorWorkDrop(const char * reason, const char * stage, EventThrottle & throttle)
{
  const std::size_t count = throttle.recordAndTakePendingCount();
  if (count == 0U) {
    return;
  }

  LogEvent(node_.get_logger(), "executor_work_dropped")
    .field("reason", reason)
    .field("stage", stage)
    .field("count", count)
    .warn();
}

void Runtime::submitToExecutor(std::function<void()> work)
{
  if (state_.shutting_down.load()) {
    logExecutorWorkDrop("shutdown", "enqueue", diagnostics_.executor_shutdown_enqueue_drop);
    return;
  }
  if (components_.ros_executor_queue == nullptr) {
    logExecutorWorkDrop("executor_unavailable", "enqueue", diagnostics_.executor_unavailable_drop);
    return;
  }
  (void)components_.ros_executor_queue->submit([this, work = std::move(work)]() mutable {
    // Work accepted before shutdown may still be draining through the queue.
    if (state_.shutting_down.load()) {
      logExecutorWorkDrop("shutdown", "execute", diagnostics_.executor_shutdown_execute_drop);
      return;
    }
    work();
  });
}

}  // namespace livekit_ros2_bridge
