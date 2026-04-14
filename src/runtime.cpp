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

#include "packet_router.hpp"
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
constexpr auto kFailFastEvaluationInterval = std::chrono::milliseconds(250);
constexpr auto kFailFastExitDelay = std::chrono::milliseconds(100);
}  // namespace

bool Runtime::State::markRpcRegistered()
{
  std::lock_guard<std::mutex> lock(mutex);
  rpc_registered = true;
  return markReadyLocked();
}

Runtime::State::ConnectTransition Runtime::State::markConnected()
{
  std::lock_guard<std::mutex> lock(mutex);
  const bool was_connected = room_connected;
  const bool was_ready_once = ready_once;
  std::string disconnect_reason = std::move(reason);
  room_connected = true;
  grace_deadline.reset();
  reason.clear();

  ConnectTransition transition;
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
, room_connection_(std::move(connection))
, config_(std::move(config))
, fail_fast_callbacks_(std::move(fail_fast_callbacks))
{
  if (room_connection_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit room connection");
  }
  LogEvent(node_.get_logger(), "runtime_startup_begin").fieldOr("room", config_.livekit.room, "<unset>").info();

  initFailFast();
  initVideoProfiling();
  initRosInterfaces();
  initSubscriptionRuntime();
  initPacketRouting();
  startRoomConnection();
}

Runtime::~Runtime()
{
  shutdown();
}

void Runtime::initFailFast()
{
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

  if (config_.health.fail_fast_enabled) {
    state_.armGraceDeadline(config_.health.fail_fast_disconnect_grace);
  }

  fail_fast_timer_ = node_.create_wall_timer(kFailFastEvaluationInterval, [this]() { checkFailFast(); });
}

void Runtime::initVideoProfiling()
{
  if (!config_.video_profiling.enabled) {
    return;
  }

  video_profiling_registry_ = std::make_unique<VideoProfilingRegistry>(node_.get_logger(), config_.video_profiling);
  video_profiling_registry_->logConfig();
  video_profile_summary_timer_ = node_.create_wall_timer(
    video_profiling_registry_->config().summary_interval, [this]() { video_profiling_registry_->logSummaries(); });
}

void Runtime::initRosInterfaces()
{
  ros_executor_queue_ = std::make_unique<RosExecutorQueue>(node_);
  ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(node_, config_.access_policy);
  ros_service_caller_ = std::make_unique<RosServiceCaller>(node_);
}

void Runtime::initSubscriptionRuntime()
{
  video_stream_registry_ = std::make_unique<VideoStreamRegistry>(
    node_, *room_connection_, &config_.subscription_qos, video_profiling_registry_.get());

  subscription_registry_ = std::make_unique<SubscriptionRegistry>(
    node_, *room_connection_, video_stream_registry_.get(), &config_.video_stream, &config_.subscription_qos);

  subscription_heartbeat_processor_ = std::make_unique<SubscriptionHeartbeatProcessor>(
    *subscription_registry_, *room_connection_, config_.access_policy, node_.get_clock());

  subscription_lease_gc_timer_ = node_.create_wall_timer(kLeaseGcInterval, [this]() {
    submitToExecutor([this]() {
      subscription_heartbeat_processor_->pruneExpiredLeases();
      subscription_registry_->pruneExpiredLeases();
    });
  });
}

void Runtime::initPacketRouting()
{
  packet_router_ = std::make_unique<PacketRouter>(
    node_.get_clock(),
    [this](std::function<void()> work) { submitToExecutor(std::move(work)); },
    *subscription_heartbeat_processor_,
    *ros_topic_publisher_);
}

void Runtime::startRoomConnection()
{
  // `start()` may deliver callbacks before RPC registration completes, so readiness is logged when
  // the second prerequisite arrives.
  room_connection_->start(
    config_.livekit,
    RoomConnectionCallbacks{
      std::bind(&Runtime::onConnected, this),
      std::bind(&Runtime::onReconnectRequested, this, std::placeholders::_1),
      std::bind(&Runtime::onConnectionReset, this),
      std::bind(&Runtime::onParticipantDisconnected, this, std::placeholders::_1),
      std::bind(&Runtime::onIncomingPacket, this, std::placeholders::_1),
    });

  rpc_router_ = std::make_unique<RpcRouter>(node_, config_.access_policy, *ros_executor_queue_, *ros_service_caller_);

  if (!rpc_router_->registerRpcs(*room_connection_)) {
    LogEvent(node_.get_logger(), "runtime_startup_failed")
      .fieldOr("room", config_.livekit.room, "<unset>")
      .field("reason", "required_rpc_registration_failed")
      .error();
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }

  if (state_.markRpcRegistered()) {
    LogEvent(node_.get_logger(), "runtime_ready").fieldOr("room", config_.livekit.room, "<unset>").info();
  }
}

void Runtime::onConnected()
{
  if (state_.shutting_down.load()) {
    return;
  }

  const auto transition = state_.markConnected();
  if (transition.became_ready) {
    LogEvent(node_.get_logger(), "runtime_ready").fieldOr("room", config_.livekit.room, "<unset>").info();
  } else if (transition.recovered) {
    LogEvent(node_.get_logger(), "runtime_reconnected")
      .fieldOr("room", config_.livekit.room, "<unset>")
      .fieldOr("disconnect_reason", transition.disconnect_reason, "connection_lost")
      .info();
  }
}

void Runtime::onReconnectRequested(const std::string & reason)
{
  if (state_.shutting_down.load()) {
    return;
  }

  const auto transition =
    state_.markDisconnected(reason, config_.health.fail_fast_enabled, config_.health.fail_fast_disconnect_grace);

  LogEvent log = LogEvent(node_.get_logger(), "runtime_disconnect_observed")
                   .fieldOr("room", config_.livekit.room, "<unset>")
                   .field("phase", transition.ready_once ? "reconnect" : "startup")
                   .fieldOr("disconnect_reason", reason, "connection_lost");

  if (!config_.health.fail_fast_enabled) {
    log.info();
    return;
  }

  log.field("grace_seconds", config_.health.fail_fast_disconnect_grace.count() / 1000.0);
  log.warn();
}

void Runtime::shutdown()
{
  if (state_.shutting_down.exchange(true)) {
    return;
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_start").fieldOr("room", config_.livekit.room, "<unset>").info();

  // Disarm periodic work first so lease GC and fail-fast evaluation stop racing a deliberate
  // shutdown while the shared components below are being torn down.
  subscription_lease_gc_timer_.reset();
  fail_fast_timer_.reset();
  video_profile_summary_timer_.reset();

  if (rpc_router_ != nullptr && room_connection_ != nullptr) {
    rpc_router_->unregisterRpcs(*room_connection_);
  }

  // Stop the room connection before shutting down the executor queue so SDK callbacks can no longer
  // enqueue fresh ROS work while already-running executor tasks finish.
  if (room_connection_ != nullptr) {
    room_connection_->stop();
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

  if (video_profiling_registry_ != nullptr) {
    video_profiling_registry_->logSummaries();
    video_profiling_registry_->flushTrace();
  }

  LogEvent(node_.get_logger(), "runtime_shutdown_complete").fieldOr("room", config_.livekit.room, "<unset>").info();
}

void Runtime::checkFailFast()
{
  if (!config_.health.fail_fast_enabled || state_.shutting_down.load()) {
    return;
  }

  const auto trigger = state_.takeFailFastTrigger(SteadyClock::now());
  if (!trigger.has_value()) {
    return;
  }

  LogEvent(node_.get_logger(), "runtime_fail_fast_triggered")
    .fieldOr("room", config_.livekit.room, "<unset>")
    .field("phase", trigger->ready_once ? "reconnect" : "startup")
    .field("disconnect_reason", trigger->reason)
    .field("grace_seconds", config_.health.fail_fast_disconnect_grace.count() / 1000.0)
    .error();

  // Give ROS shutdown and log flushing a brief head start before forcing process exit.
  fail_fast_callbacks_.shutdown_callback();
  std::this_thread::sleep_for(kFailFastExitDelay);
  fail_fast_callbacks_.exit_callback(EXIT_FAILURE);
}

void Runtime::onConnectionReset()
{
  submitToExecutor([this]() {
    // Reset session-owned state on the ROS executor so cleanup stays ordered with any
    // in-flight heartbeat or RPC work targeting the old connection generation.
    subscription_registry_->resetSessionState();
    ros_service_caller_->resetSessionState();
  });
}

void Runtime::onParticipantDisconnected(std::string requester_identity)
{
  const std::size_t generation = subscription_registry_->generation();
  submitToExecutor([this, requester_identity = std::move(requester_identity), generation]() {
    // Keep leases alive across a browser refresh, but queue fresh data-track publications
    // because LiveKit binds them to the old participant_session.
    subscription_registry_->queueDataTrackRepublish(requester_identity, generation);
    ros_service_caller_->cancelCallsForRequester(requester_identity);
  });
}

void Runtime::onIncomingPacket(const IncomingPacket & packet)
{
  if (state_.shutting_down.load()) {
    logPacketDrop(packet, "shutdown", diagnostics_.packet_shutdown_drop);
    return;
  }
  if (packet_router_ == nullptr) {
    logPacketDrop(packet, "router_unavailable", diagnostics_.packet_router_unavailable_drop);
    return;
  }
  packet_router_->handle(packet);
}

void Runtime::logPacketDrop(const IncomingPacket & packet, const char * reason, EventThrottle & throttle) const
{
  const std::size_t count = throttle.recordAndTakePendingCount();
  if (count == 0U) {
    return;
  }

  LogEvent(node_.get_logger(), "packet_dropped")
    .field("reason", reason)
    .field("topic", packet.topic)
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

  if (ros_executor_queue_ == nullptr) {
    logExecutorWorkDrop("executor_unavailable", "enqueue", diagnostics_.executor_unavailable_drop);
    return;
  }

  (void)ros_executor_queue_->submit([this, work = std::move(work)]() mutable {
    // Work accepted before shutdown may still be draining through the queue.
    if (state_.shutting_down.load()) {
      logExecutorWorkDrop("shutdown", "execute", diagnostics_.executor_shutdown_execute_drop);
      return;
    }
    work();
  });
}

}  // namespace livekit_ros2_bridge
