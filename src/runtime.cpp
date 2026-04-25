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
#include <utility>

#include "livekit/remote_participant.h"
#include "livekit/room_event_types.h"
#include "protocol/constants.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

Runtime::Runtime(RuntimeNodeInterfaces interfaces, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
: clock_(interfaces.get_node_clock_interface()->get_clock())
, logger_(interfaces.get_node_logging_interface()->get_logger())
, config_(std::move(config))
, room_connection_(std::move(connection))
, ros_executor_queue_(interfaces, clock_)
, ros_topic_publisher_(
    interfaces.get_node_topics_interface(), interfaces.get_node_graph_interface(), clock_, config_.access_policy)
, ros_service_caller_(
    interfaces.get_node_base_interface(),
    interfaces.get_node_graph_interface(),
    interfaces.get_node_waitables_interface())
, subscription_lease_manager_(
    interfaces.get_node_parameters_interface(),
    interfaces.get_node_topics_interface(),
    interfaces.get_node_graph_interface(),
    clock_,
    *room_connection_,
    config_.access_policy,
    &config_.subscription_qos,
    &config_.video_stream)
, rpc_router_(interfaces.get_node_graph_interface(), config_.access_policy, ros_executor_queue_, ros_service_caller_)
, watchdog_(config_.health, interfaces, [this]() { return callback_gate_.closeAndWait(); })
{
  subscription_lease_manager_.startLeaseGcTimer(
    interfaces.get_node_base_interface(), interfaces.get_node_timers_interface(), [this](std::function<void()> work) {
      submitToExecutor(std::move(work));
    });

  const bool rpcs_registered = rpc_router_.registerRpcs(*room_connection_);
  if (!rpcs_registered) {
    throw std::runtime_error("Failed to register required RPC methods");
  }

  room_connection_->start(config_.livekit, makeRoomEventCallbacks());
}

Runtime::~Runtime()
{
  if (callback_gate_.closeAndWait()) {
    LogEvent(logger_, "runtime_shutdown_start").info();
  }

  ros_executor_queue_.shutdown();
  subscription_lease_manager_.shutdown();
  rpc_router_.unregisterRpcs();
  room_connection_->stop();
}

RoomEventCallbacks Runtime::makeRoomEventCallbacks()
{
  RoomEventCallbacks callbacks;
  callbacks.on_connection_state_changed = [this](livekit::ConnectionState state) {
    (void)callback_gate_.runIfOpen([this, state]() { watchdog_.observeConnectionState(state); });
  };
  callbacks.on_user_packet_received = [this](const livekit::UserDataPacketEvent & event) {
    (void)callback_gate_.runIfOpen([this, &event]() { onRoomUserPacketReceived(event); });
  };
  callbacks.on_participant_disconnected = [this](const livekit::ParticipantDisconnectedEvent & event) {
    (void)callback_gate_.runIfOpen([this, &event]() {
      std::string remote_participant_identity = event.participant->identity();
      submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
        subscription_lease_manager_.onRemoteParticipantDisconnected(remote_participant_identity);
        ros_service_caller_.cancelForRequester(remote_participant_identity);
      });
    });
  };

  return callbacks;
}

bool RuntimeCallbackGate::closeAndWait()
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool closed_by_caller = !closed_;
  closed_ = true;
  idle_.wait(lock, [this]() { return active_count_ == 0U; });
  return closed_by_caller;
}

void Runtime::onRoomUserPacketReceived(const livekit::UserDataPacketEvent & event)
{
  const std::string topic = event.topic;
  const std::string requester_identity = event.participant == nullptr ? "" : event.participant->identity();

  // SDK event and participant lifetimes do not extend to queued ROS work.
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
    .fieldOr("topic", topic)
    .fieldOr("requester_identity", requester_identity)
    .warnThrottle(*clock_, std::chrono::seconds(5));
}

void Runtime::submitToExecutor(std::function<void()> work)
{
  (void)callback_gate_.runIfOpen(
    [this, work = std::move(work)]() mutable { (void)ros_executor_queue_.submit(std::move(work)); });
}

}  // namespace livekit_ros2_bridge
