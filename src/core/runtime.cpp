// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/runtime.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "audio/audio_output_manager.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "livekit/remote_participant.h"
#include "livekit/room_event_types.h"
#include "protocol/constants.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_timer.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kDiagnosticsPeriod = std::chrono::milliseconds(1000);

template <typename StreamConfig>
std::vector<std::string> otherSourceIds(const StreamConfig & config)
{
  std::vector<std::string> ids;
  ids.reserve(config.other_sources.size());
  for (const auto & source : config.other_sources) {
    ids.push_back(source.name);
  }
  return ids;
}

CapabilityFeatures makeCapabilityFeatures(const RuntimeConfig & config)
{
  CapabilityFeatures features;
  features.audio_output_enabled = !config.audio_output.sink_fragment.empty();
  features.audio_other_ids = otherSourceIds(config.audio_stream);
  features.video_other_ids = otherSourceIds(config.video_stream);
  return features;
}

uint8_t diagnosticLevel(livekit::ConnectionState state)
{
  switch (state) {
    case livekit::ConnectionState::Connected:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case livekit::ConnectionState::Reconnecting:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case livekit::ConnectionState::Disconnected:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::STALE;
}

}  // namespace

Runtime::Runtime(Runtime::NodeInterfaces interfaces, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
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
    &config_.video_stream,
    &config_.audio_stream)
, rpc_router_(
    interfaces.get_node_graph_interface(),
    config_.access_policy,
    ros_executor_queue_,
    ros_service_caller_,
    subscription_lease_manager_,
    makeCapabilityFeatures(config_))
, watchdog_(config_.watchdog, logger_)
{
  subscription_lease_manager_.startPruneTimer(
    interfaces.get_node_base_interface(), interfaces.get_node_timers_interface(), [this](std::function<void()> work) {
      submitRosWork(std::move(work));
    });

  // Created only when a sink is configured, so an unconfigured bridge ignores track events.
  if (!config_.audio_output.sink_fragment.empty()) {
    audio_output_manager_ =
      std::make_unique<audio::AudioOutputManager>(*room_connection_, config_.audio_output.sink_fragment);
    LogEvent(logger_, "audio_out_enabled").info();
  }

  diagnostics_publisher_ =
    rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(interfaces, "/diagnostics", rclcpp::QoS(10));
  diagnostics_timer_ = rclcpp::create_wall_timer(
    kDiagnosticsPeriod,
    [this]() { (void)callback_gate_.run([this]() { publishDiagnostics(); }); },
    nullptr,
    interfaces.get_node_base_interface().get(),
    interfaces.get_node_timers_interface().get());

  const bool rpcs_registered = rpc_router_.registerRpcs(*room_connection_);
  if (!rpcs_registered) {
    throw std::runtime_error("Failed to register required RPC methods");
  }

  room_connection_->start(config_.livekit, makeRoomCallbacks());
}

Runtime::~Runtime()
{
  if (callback_gate_.closeAndWait()) {
    LogEvent(logger_, "node_shutdown_start").info();
  }

  watchdog_.stop();
  ros_executor_queue_.shutdown();
  subscription_lease_manager_.shutdown();
  rpc_router_.unregisterRpcs();
  // Destroy the audio output manager before the room stops: it flips every
  // reader's stop flag and closes the streams so no reader thread outlives
  // this Runtime.
  audio_output_manager_.reset();
  room_connection_->stop();
}

RoomEventCallbacks Runtime::makeRoomCallbacks()
{
  RoomEventCallbacks callbacks;
  callbacks.on_state_changed = [this](livekit::ConnectionState state) {
    (void)callback_gate_.run([this, state]() { onConnectionStateChanged(state); });
  };
  callbacks.on_user_packet_received = [this](const livekit::UserDataPacketEvent & event) {
    (void)callback_gate_.run([this, &event]() { onUserPacketReceived(event); });
  };
  callbacks.on_participant_disconnected = [this](const livekit::ParticipantDisconnectedEvent & event) {
    (void)callback_gate_.run([this, &event]() {
      if (audio_output_manager_ != nullptr) {
        audio_output_manager_->onParticipantDisconnected(event);
      }
      std::string identity = event.participant->identity();
      submitRosWork([this, identity = std::move(identity)]() { ros_service_caller_.cancelForRequester(identity); });
    });
  };

  // Audio output track events run on SDK delegate threads and are wrapped in
  // callback_gate_ like every other callback. They only log, subscribe, and
  // start or stop the reader threads that feed the sink; no ROS work is submitted.
  if (audio_output_manager_ != nullptr) {
    audio::AudioOutputManager * audio_output_manager = audio_output_manager_.get();
    // Readers survive a reconnect; catching up only subscribes output tracks
    // the snapshot reports as not yet subscribed, including re-announced ones.
    callbacks.on_remote_tracks_ready = [this, audio_output_manager]() {
      (void)callback_gate_.run([audio_output_manager]() { audio_output_manager->onConnected(); });
    };
    callbacks.on_remote_track_published = [this, audio_output_manager](const RemoteTrackEvent & event) {
      (void)callback_gate_.run(
        [audio_output_manager, &event]() { audio_output_manager->onRemoteTrackPublished(event); });
    };
    callbacks.on_remote_track_unpublished = [this, audio_output_manager](const RemoteTrackEvent & event) {
      (void)callback_gate_.run(
        [audio_output_manager, &event]() { audio_output_manager->onRemoteTrackUnpublished(event); });
    };
    callbacks.on_remote_track_subscribed = [this, audio_output_manager](const RemoteTrackEvent & event) {
      (void)callback_gate_.run(
        [audio_output_manager, &event]() { audio_output_manager->onRemoteTrackSubscribed(event); });
    };
    callbacks.on_remote_track_unsubscribed = [this, audio_output_manager](const RemoteTrackEvent & event) {
      (void)callback_gate_.run(
        [audio_output_manager, &event]() { audio_output_manager->onRemoteTrackUnsubscribed(event); });
    };
    callbacks.on_remote_track_subscription_failed =
      [this, audio_output_manager](const RemoteTrackSubscriptionFailedEvent & event) {
        (void)callback_gate_.run(
          [audio_output_manager, &event]() { audio_output_manager->onRemoteTrackSubscriptionFailed(event); });
      };
  }

  return callbacks;
}

void Runtime::onUserPacketReceived(const livekit::UserDataPacketEvent & event)
{
  const std::string topic = event.topic;
  const std::string requester = event.participant == nullptr ? "" : event.participant->identity();

  // SDK event and participant lifetimes do not extend to queued ROS work.
  if (topic == protocol::kPublishRequestTopic) {
    submitRosWork(
      [this, requester, payload = event.data]() { ros_topic_publisher_.handlePayload(requester, payload); });
    return;
  }

  if (topic == protocol::kHeartbeatTopic) {
    submitRosWork([this, requester, payload = event.data]() {
      subscription_lease_manager_.handleHeartbeatPayload(requester, payload);
    });
    return;
  }

  LogEvent(logger_, "livekit_packet_dropped")
    .field("reason", "unsupported_topic")
    .fieldOr("topic", topic)
    .fieldOr("requester_identity", requester)
    .warnThrottle(*clock_, std::chrono::seconds(5));
}

void Runtime::onConnectionStateChanged(livekit::ConnectionState state)
{
  watchdog_.onStateChanged(state);
  connection_state_ = state;
  LogEvent(logger_, "connection_state_changed").fieldEnum("state", state).info();
  publishDiagnostics();
}

void Runtime::publishDiagnostics()
{
  const auto state = connection_state_.load();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnosticLevel(state);
  status.name = "livekit_ros2_bridge: connection";
  status.message = std::string(magic_enum::enum_name(state));
  status.hardware_id = "livekit";

  diagnostic_msgs::msg::KeyValue connection_state;
  connection_state.key = "connection_state";
  connection_state.value = status.message;
  status.values.push_back(std::move(connection_state));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

void Runtime::submitRosWork(std::function<void()> work)
{
  (void)callback_gate_.run(
    [this, work = std::move(work)]() mutable { (void)ros_executor_queue_.submit(std::move(work)); });
}

}  // namespace livekit_ros2_bridge
