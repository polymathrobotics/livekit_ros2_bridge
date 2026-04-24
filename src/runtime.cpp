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

#include "livekit/remote_participant.h"
#include "livekit/room_event_types.h"
#include "livekit_room_delegate.hpp"
#include "protocol/constants.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::unique_ptr<RoomConnection> requireRoomConnection(std::unique_ptr<RoomConnection> connection)
{
  if (!connection) {
    throw std::invalid_argument("Runtime requires a non-null RoomConnection");
  }
  return connection;
}

}  // namespace

RuntimeNodeInterfaces RuntimeNodeInterfaces::fromNode(rclcpp::Node & node)
{
  return RuntimeNodeInterfaces{
    node.get_node_base_interface(),
    node.get_node_graph_interface(),
    node.get_node_topics_interface(),
    node.get_node_waitables_interface(),
    node.get_node_timers_interface(),
    node.get_node_parameters_interface(),
    node.get_clock(),
    node.get_logger(),
  };
}

Runtime::Runtime(RuntimeNodeInterfaces interfaces, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
: base_(std::move(interfaces.base))
, graph_(std::move(interfaces.graph))
, timers_(std::move(interfaces.timers))
, clock_(std::move(interfaces.clock))
, logger_(interfaces.logger)
, config_(std::move(config))
, room_delegate_(std::make_unique<LiveKitRoomDelegate>(makeRoomEventCallbacks()))
, room_connection_(requireRoomConnection(std::move(connection)))
, ros_executor_queue_(base_, interfaces.waitables, clock_)
, ros_topic_publisher_(interfaces.topics, graph_, clock_, config_.access_policy)
, ros_service_caller_(base_, graph_, interfaces.waitables)
, subscription_lease_manager_(
    interfaces.parameters,
    interfaces.topics,
    graph_,
    clock_,
    *room_connection_,
    config_.access_policy,
    &config_.subscription_qos,
    &config_.video_stream)
, rpc_router_(graph_, config_.access_policy, ros_executor_queue_, ros_service_caller_)
, watchdog_(config_.health, base_, timers_, logger_, [this]() { return callback_gate_.closeAndWait(); })
{
  LogEvent(logger_, "runtime_startup_begin")
    .fieldOr("url", config_.livekit.url, "<unset>")
    .field("token_present", !config_.livekit.access_token.empty())
    .info();

  subscription_lease_manager_.startLeaseGcTimer(
    base_, timers_, [this](std::function<void()> work) { submitToExecutor(std::move(work)); });

  const bool rpcs_registered = rpc_router_.registerRpcs(*room_connection_);
  if (!rpcs_registered) {
    throw std::runtime_error("Failed to register required RPC methods");
  }

  room_connection_->start(config_.livekit, *room_delegate_);
}

Runtime::~Runtime()
{
  if (callback_gate_.closeAndWait()) {
    LogEvent(logger_, "runtime_shutdown_start").info();
  }

  ros_executor_queue_.shutdown();
}

RoomEventCallbacks Runtime::makeRoomEventCallbacks()
{
  RoomEventCallbacks callbacks;
  callbacks.on_connected = [this]() { (void)callback_gate_.runIfOpen([this]() { onRoomConnected(); }); };
  callbacks.on_user_packet_received = [this](const livekit::UserDataPacketEvent & event) {
    const bool handled = callback_gate_.runIfOpen([this, &event]() { onRoomUserPacketReceived(event); });
    if (handled) {
      return;
    }

    LogEvent(logger_, "packet_dropped")
      .field("reason", "shutdown")
      .field("topic", event.topic)
      .fieldOr("requester_identity", event.participant == nullptr ? "" : event.participant->identity())
      .warnThrottle(*clock_, std::chrono::seconds(5));
  };
  callbacks.on_participant_disconnected = [this](const std::string & remote_participant_identity) {
    (void)callback_gate_.runIfOpen(
      [this, &remote_participant_identity]() { onRoomRemoteParticipantDisconnected(remote_participant_identity); });
  };
  callbacks.on_reconnect_requested = [this](const std::string & reason) {
    (void)callback_gate_.runIfOpen([this, &reason]() { onRoomReconnectRequested(reason); });
  };
  callbacks.on_reconnecting = [this](const std::string & reason) {
    (void)callback_gate_.runIfOpen([this, &reason]() { onRoomReconnecting(reason); });
  };
  callbacks.on_reconnected = [this]() { (void)callback_gate_.runIfOpen([this]() { onRoomReconnected(); }); };
  callbacks.on_connection_reset = [this]() { (void)callback_gate_.runIfOpen([this]() { onRoomConnectionReset(); }); };

  return callbacks;
}

bool RuntimeCallbackGate::tryEnter()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return false;
  }

  ++active_count_;
  return true;
}

void RuntimeCallbackGate::leave()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    --active_count_;
  }

  idle_.notify_all();
}

bool RuntimeCallbackGate::closeAndWait()
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool closed_by_caller = !closed_;
  closed_ = true;
  idle_.wait(lock, [this]() { return active_count_ == 0U; });
  return closed_by_caller;
}

void Runtime::onRoomConnected()
{
  watchdog_.markHealthy("room_connected");
  LogEvent(logger_, "runtime_ready").info();
}

void Runtime::onRoomUserPacketReceived(const livekit::UserDataPacketEvent & event)
{
  const std::string topic = event.topic;
  const std::string requester_identity = event.participant == nullptr ? "" : event.participant->identity();

  // LiveKit owns the event lifetime, so executor work must capture only copied fields.
  if (topic == protocol::kRosPublishTopic) {
    submitToExecutor([this, requester_identity, payload = event.data]() {
      ros_topic_publisher_.handlePublishPayload(requester_identity, payload);
    });
    return;
  }

  if (topic == protocol::kHeartbeatTopic) {
    submitToExecutor([this, requester_identity, payload = event.data]() {
      subscription_lease_manager_.handleHeartbeatPayload(requester_identity, payload);
    });
    return;
  }

  LogEvent(logger_, "packet_dropped")
    .field("reason", "unsupported_topic")
    .field("topic", topic)
    .fieldOr("requester_identity", requester_identity)
    .warnThrottle(*clock_, std::chrono::seconds(5));
}

void Runtime::onRoomRemoteParticipantDisconnected(std::string remote_participant_identity)
{
  submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
    subscription_lease_manager_.onRemoteParticipantDisconnected(remote_participant_identity);
    ros_service_caller_.cancelForRequester(remote_participant_identity);
  });
}

void Runtime::onRoomReconnectRequested(const std::string & reason)
{
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
  watchdog_.markHealthy("room_reconnected");
  LogEvent(logger_, "runtime_ready").info();
}

void Runtime::onRoomConnectionReset()
{
  submitToExecutor([this]() {
    subscription_lease_manager_.resetSessionState();
    ros_service_caller_.resetSessionState();
  });
}

void Runtime::submitToExecutor(std::function<void()> work)
{
  (void)callback_gate_.runIfOpen(
    [this, work = std::move(work)]() mutable { (void)ros_executor_queue_.submit(std::move(work)); });
}

}  // namespace livekit_ros2_bridge
