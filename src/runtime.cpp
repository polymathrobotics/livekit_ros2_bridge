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
#include <stdexcept>
#include <string_view>
#include <utility>

#include "rclcpp/create_timer.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);

}  // namespace

Runtime::ShutdownLogScope::ShutdownLogScope(rclcpp::Logger logger)
: logger_(std::move(logger))
{}

void Runtime::ShutdownLogScope::begin()
{
  if (started_) {
    return;
  }

  started_ = true;
  LogEvent(logger_, "runtime_shutdown_start").info();
}

Runtime::ShutdownLogScope::~ShutdownLogScope()
{
  if (!started_) {
    return;
  }

  LogEvent(logger_, "runtime_shutdown_complete").info();
}

Runtime::Runtime(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
: shutdown_log_scope_(node.get_logger())
, base_(node.get_node_base_interface())
, graph_(node.get_node_graph_interface())
, timers_(node.get_node_timers_interface())
, clock_(node.get_clock())
, logger_(node.get_logger())
, config_(std::move(config))
, room_connection_(std::move(connection))
, ros_executor_queue_(base_, node.get_node_waitables_interface(), clock_)
, ros_topic_publisher_(node.get_node_topics_interface(), graph_, clock_, config_.access_policy)
, ros_service_caller_(base_, graph_, node.get_node_waitables_interface())
, subscription_lease_manager_(
    node.get_node_parameters_interface(),
    node.get_node_topics_interface(),
    graph_,
    clock_,
    room_connection_.connection(),
    config_.access_policy,
    &config_.subscription_qos,
    &config_.video_stream)
, packet_router_(
    clock_,
    [this](std::function<void()> work) { submitToExecutor(std::move(work)); },
    subscription_lease_manager_,
    ros_topic_publisher_)
, rpc_router_(graph_, config_.access_policy, ros_executor_queue_, ros_service_caller_)
, watchdog_(config_.health, base_, timers_, logger_, [this]() { return closeCallbacks(); })
{
  LogEvent(logger_, "runtime_startup_begin")
    .fieldOr("url", config_.livekit.url, "<unset>")
    .field("token_present", !config_.livekit.access_token.empty())
    .info();

  subscription_lease_gc_timer_ = rclcpp::create_wall_timer(
    kLeaseGcInterval,
    [this]() { submitToExecutor([this]() { subscription_lease_manager_.pruneExpiredLeases(); }); },
    nullptr,
    base_.get(),
    timers_.get());

  const bool rpcs_registered = rpc_router_.registerRpcs(room_connection_.connection());
  if (!rpcs_registered) {
    LogEvent(logger_, "runtime_startup_failed")
      .fieldOr("url", config_.livekit.url, "<unset>")
      .field("token_present", !config_.livekit.access_token.empty())
      .field("reason", "required_rpc_registration_failed")
      .error();
    throw std::runtime_error("Failed to register required RPC methods");
  }

  RoomEventCallbacks callbacks;
  callbacks.on_connected = std::bind(&Runtime::onRoomConnected, this);
  callbacks.on_incoming_packet_received = std::bind(&Runtime::onRoomIncomingPacket, this, std::placeholders::_1);
  callbacks.on_remote_participant_disconnected =
    std::bind(&Runtime::onRoomRemoteParticipantDisconnected, this, std::placeholders::_1);
  callbacks.on_reconnect_requested = std::bind(&Runtime::onRoomReconnectRequested, this, std::placeholders::_1);
  callbacks.on_reconnecting = std::bind(&Runtime::onRoomReconnecting, this, std::placeholders::_1);
  callbacks.on_reconnected = std::bind(&Runtime::onRoomReconnected, this);
  callbacks.on_connection_reset = std::bind(&Runtime::onRoomConnectionReset, this);

  room_connection_.connection().start(config_.livekit, std::move(callbacks));
}

Runtime::~Runtime()
{
  if (!closeCallbacks()) {
    return;
  }

  shutdown_log_scope_.begin();

  subscription_lease_gc_timer_.reset();

  ros_executor_queue_.shutdown();
}

Runtime::ScopedRoomConnection::ScopedRoomConnection(std::unique_ptr<RoomConnection> connection)
: connection_(std::move(connection))
{
  if (connection_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit room connection");
  }
}

Runtime::ScopedRoomConnection::~ScopedRoomConnection()
{
  connection_->stop();
}

RoomConnection & Runtime::ScopedRoomConnection::connection() const
{
  return *connection_;
}

bool Runtime::closeCallbacks()
{
  bool expected = false;
  return callbacks_closed_.compare_exchange_strong(expected, true);
}

bool Runtime::callbacksClosed() const
{
  return callbacks_closed_.load();
}

void Runtime::onRoomConnected()
{
  if (callbacksClosed()) {
    return;
  }

  watchdog_.markHealthy("room_connected");
  LogEvent(logger_, "runtime_ready").info();
}

void Runtime::onRoomIncomingPacket(const IncomingPacket & packet)
{
  if (callbacksClosed()) {
    LogEvent(logger_, "packet_dropped")
      .field("reason", "shutdown")
      .field("topic", packet.topic)
      .fieldOr("requester_identity", packet.requester_identity)
      .warnThrottle(*clock_, std::chrono::seconds(5));
    return;
  }

  packet_router_.handle(packet);
}

void Runtime::onRoomRemoteParticipantDisconnected(std::string remote_participant_identity)
{
  if (callbacksClosed()) {
    return;
  }

  submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
    subscription_lease_manager_.onRemoteParticipantDisconnected(remote_participant_identity);
    ros_service_caller_.cancelForRequester(remote_participant_identity);
  });
}

void Runtime::onRoomReconnectRequested(const std::string & reason)
{
  if (callbacksClosed()) {
    return;
  }

  const std::string_view disconnect_reason =
    reason.empty() ? std::string_view("connection_lost") : std::string_view(reason);
  watchdog_.markUnhealthy(disconnect_reason);

  LogEvent log = LogEvent(logger_, "runtime_disconnect_observed").field("disconnect_reason", disconnect_reason);

  if (!config_.health.watchdog_enabled) {
    log.info();
    return;
  }

  log.field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0);
  log.warn();
}

void Runtime::onRoomReconnecting(const std::string & reason)
{
  onRoomReconnectRequested(reason);
}

void Runtime::onRoomReconnected()
{
  if (callbacksClosed()) {
    return;
  }

  watchdog_.markHealthy("room_reconnected");
  LogEvent(logger_, "runtime_ready").info();
}

void Runtime::onRoomConnectionReset()
{
  if (callbacksClosed()) {
    return;
  }

  submitToExecutor([this]() {
    subscription_lease_manager_.resetSessionState();
    ros_service_caller_.resetSessionState();
  });
}

void Runtime::submitToExecutor(std::function<void()> work)
{
  if (callbacksClosed()) {
    return;
  }

  (void)ros_executor_queue_.submit([work = std::move(work)]() mutable { work(); });
}

}  // namespace livekit_ros2_bridge
