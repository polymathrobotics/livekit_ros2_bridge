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

#include "data_stream_registry.hpp"
#include "packet_router.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "subscription_lease_manager.hpp"
#include "topic_publish_command.hpp"
#include "utils/log_event.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr auto kWatchdogEvaluationInterval = std::chrono::milliseconds(250);
constexpr auto kShutdownExitDelay = std::chrono::milliseconds(100);
}  // namespace

Runtime::Runtime(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
: node_(node)
, config_(std::move(config))
, room_connection_(std::move(connection))
{
  if (room_connection_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit room connection");
  }
  LogEvent(node_.get_logger(), "runtime_startup_begin").fieldOr("room", config_.livekit.room, "<unset>").info();

  // Preflight: arm shutdown watchdog monitoring before transport startup begins.
  if (config_.health.watchdog_enabled) {
    watchdog_deadline_ = SteadyClock::now() + config_.health.watchdog_recovery_timeout;
  }
  watchdog_timer_ = node_.create_wall_timer(kWatchdogEvaluationInterval, [this]() { checkWatchdog(); });

  // Preflight: configure optional video profiling before any stream state is created.
  if (config_.video_profiling.enabled) {
    video_profiling_registry_ = std::make_unique<VideoProfilingRegistry>(node_.get_logger(), config_.video_profiling);
    video_profiling_registry_->logConfig();
    video_profile_summary_timer_ = node_.create_wall_timer(
      video_profiling_registry_->config().summary_interval, [this]() { video_profiling_registry_->logSummaries(); });
  }

  // Bring up the core ROS-facing helpers first. Later handlers and session state depend on these.
  ros_executor_queue_ = std::make_unique<RosExecutorQueue>(node_);
  ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(node_, config_.access_policy);
  ros_service_caller_ = std::make_unique<RosServiceCaller>(node_);

  // Build the session-owned subscription and stream state on top of the room connection.
  data_stream_registry_ = std::make_unique<DataStreamRegistry>(node_, *room_connection_, &config_.subscription_qos);

  video_stream_registry_ = std::make_unique<VideoStreamRegistry>(
    node_, *room_connection_, &config_.subscription_qos, video_profiling_registry_.get(), &config_.video_stream);

  subscription_lease_manager_ = std::make_unique<SubscriptionLeaseManager>(
    node_, *room_connection_, config_.access_policy, *data_stream_registry_, *video_stream_registry_);

  subscription_lease_gc_timer_ = node_.create_wall_timer(
    kLeaseGcInterval, [this]() { submitToExecutor([this]() { subscription_lease_manager_->pruneExpiredLeases(); }); });

  // Build ingress and control-plane handlers before the room starts delivering callbacks.
  packet_router_ = std::make_unique<PacketRouter>(
    node_.get_clock(),
    [this](std::function<void()> work) { submitToExecutor(std::move(work)); },
    *subscription_lease_manager_,
    *ros_topic_publisher_);

  rpc_router_ = std::make_unique<RpcRouter>(node_, config_.access_policy, *ros_executor_queue_, *ros_service_caller_);

  // Finish transport startup last: register required RPCs before `start()`, then expose the room
  // callbacks only after every ingress path above is ready.
  if (!rpc_router_->registerRpcs(*room_connection_)) {
    LogEvent(node_.get_logger(), "runtime_startup_failed")
      .fieldOr("room", config_.livekit.room, "<unset>")
      .field("reason", "required_rpc_registration_failed")
      .error();
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }

  // Expose the full LiveKit room callback surface only after every ingress path above is ready.
  room_connection_->start(
    config_.livekit,
    RoomEventCallbacks{
      std::bind(&Runtime::onRoomConnected, this),
      std::bind(&Runtime::onRoomIncomingPacket, this, std::placeholders::_1),
      std::bind(&Runtime::onRoomRemoteParticipantDisconnected, this, std::placeholders::_1),
      std::bind(&Runtime::onRoomReconnectRequested, this, std::placeholders::_1),
      std::bind(&Runtime::onRoomConnectionReset, this),
    });
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

  LogEvent(node_.get_logger(), "runtime_shutdown_start").fieldOr("room", config_.livekit.room, "<unset>").info();

  // Disarm periodic work first so lease GC and watchdog evaluation stop racing a deliberate
  // shutdown while the shared components below are being torn down.
  subscription_lease_gc_timer_.reset();
  watchdog_timer_.reset();
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

  if (subscription_lease_manager_ != nullptr) {
    subscription_lease_manager_->shutdown();
  }

  if (data_stream_registry_ != nullptr) {
    data_stream_registry_->shutdown();
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

void Runtime::onRoomConnected()
{
  if (shutting_down_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(watchdog_mutex_);
    watchdog_deadline_.reset();
  }

  LogEvent(node_.get_logger(), "runtime_ready").fieldOr("room", config_.livekit.room, "<unset>").info();
}

void Runtime::onRoomIncomingPacket(const IncomingPacket & packet)
{
  if (shutting_down_.load()) {
    LogEvent(node_.get_logger(), "packet_dropped")
      .field("reason", "shutdown")
      .field("topic", packet.topic)
      .fieldOr("requester_identity", packet.requester_identity)
      .warnThrottle(*node_.get_clock(), std::chrono::seconds(5));

    return;
  }

  if (packet_router_ == nullptr) {
    LogEvent(node_.get_logger(), "packet_dropped")
      .field("reason", "router_unavailable")
      .field("topic", packet.topic)
      .fieldOr("requester_identity", packet.requester_identity)
      .warnThrottle(*node_.get_clock(), std::chrono::seconds(5));

    return;
  }

  packet_router_->handle(packet);
}

void Runtime::onRoomRemoteParticipantDisconnected(std::string remote_participant_identity)
{
  submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
    // Keep leases alive across a browser refresh, but queue fresh data-track publications
    // because LiveKit binds them to the old participant_session.
    subscription_lease_manager_->onRemoteParticipantDisconnected(remote_participant_identity);
    ros_service_caller_->cancelCallsForRequester(remote_participant_identity);
  });
}

void Runtime::onRoomReconnectRequested(const std::string & reason)
{
  if (shutting_down_.load()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(watchdog_mutex_);
    if (config_.health.watchdog_enabled) {
      watchdog_deadline_ = SteadyClock::now() + config_.health.watchdog_recovery_timeout;
    } else {
      watchdog_deadline_.reset();
    }
  }

  LogEvent log = LogEvent(node_.get_logger(), "runtime_disconnect_observed")
                   .fieldOr("room", config_.livekit.room, "<unset>")
                   .fieldOr("disconnect_reason", reason, "connection_lost");

  if (!config_.health.watchdog_enabled) {
    log.info();
    return;
  }

  log.field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0);
  log.warn();
}

void Runtime::onRoomConnectionReset()
{
  submitToExecutor([this]() {
    // Reset session-owned state on the ROS executor so cleanup stays ordered with any
    // in-flight heartbeat or RPC work targeting the old connection generation.
    subscription_lease_manager_->resetSessionState();
    ros_service_caller_->resetSessionState();
  });
}

void Runtime::submitToExecutor(std::function<void()> work)
{
  if (shutting_down_.load()) {
    if (const std::size_t count = executor_shutdown_enqueue_drop_.recordAndTakePendingCount(); count > 0U) {
      LogEvent(node_.get_logger(), "executor_work_dropped")
        .field("reason", "shutdown")
        .field("stage", "enqueue")
        .field("count", count)
        .warn();
    }

    return;
  }

  if (ros_executor_queue_ != nullptr) {
    (void)ros_executor_queue_->submit([this, work = std::move(work)]() mutable {
      // Work accepted before shutdown may still be draining through the queue.
      if (shutting_down_.load()) {
        if (const std::size_t count = executor_shutdown_execute_drop_.recordAndTakePendingCount(); count > 0U) {
          LogEvent(node_.get_logger(), "executor_work_dropped")
            .field("reason", "shutdown")
            .field("stage", "execute")
            .field("count", count)
            .warn();
        }
        return;
      }

      work();
    });
    return;
  }

  if (const std::size_t count = executor_unavailable_drop_.recordAndTakePendingCount(); count > 0U) {
    LogEvent(node_.get_logger(), "executor_work_dropped")
      .field("reason", "executor_unavailable")
      .field("stage", "enqueue")
      .field("count", count)
      .warn();
  }
}

void Runtime::checkWatchdog()
{
  if (!config_.health.watchdog_enabled || shutting_down_.load()) {
    return;
  }

  const auto now = SteadyClock::now();
  {
    std::lock_guard<std::mutex> lock(watchdog_mutex_);
    if (!watchdog_deadline_.has_value()) {
      return;
    }
    if (now < *watchdog_deadline_) {
      return;
    }
  }

  if (shutting_down_.exchange(true)) {
    return;
  }

  LogEvent(node_.get_logger(), "runtime_watchdog_triggered")
    .fieldOr("room", config_.livekit.room, "<unset>")
    .field("disconnect_reason", "recovery_timeout")
    .field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0)
    .error();

  // Give ROS shutdown and log flushing a brief head start before forcing process exit.
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  std::this_thread::sleep_for(kShutdownExitDelay);
  std::_Exit(EXIT_FAILURE);
}

}  // namespace livekit_ros2_bridge
