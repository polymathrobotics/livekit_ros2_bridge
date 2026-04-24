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

#include "livekit_room_delegate.hpp"

#include <utility>

#include "livekit/remote_participant.h"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.room_connection");

}  // namespace

LiveKitRoomDelegate::LiveKitRoomDelegate(RoomEventCallbacks callbacks)
: callbacks_(std::move(callbacks))
{}

void LiveKitRoomDelegate::setReconnectRequestHandler(ReconnectRequestHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  request_reconnect_ = std::move(handler);
}

void LiveKitRoomDelegate::clearReconnectRequestHandler()
{
  std::lock_guard<std::mutex> lock(mutex_);
  request_reconnect_ = nullptr;
  state_ = livekit::ConnectionState::Disconnected;
}

void LiveKitRoomDelegate::roomConnected()
{
  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = livekit::ConnectionState::Connected;
    callback = callbacks_.on_connected;
  }

  if (callback) {
    callback();
  }
}

void LiveKitRoomDelegate::roomConnectionReset()
{
  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = livekit::ConnectionState::Disconnected;
    callback = callbacks_.on_connection_reset;
  }

  if (callback) {
    callback();
  }
}

void LiveKitRoomDelegate::requestReconnect(const std::string & reason)
{
  std::function<bool(const std::string &)> handler;
  std::function<void(const std::string &)> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = livekit::ConnectionState::Disconnected;
    handler = request_reconnect_;
    callback = callbacks_.on_reconnect_requested;
  }

  if (!handler || !handler(reason)) {
    return;
  }

  LogEvent(kLogger, "room_reconnect_requested").field("reason", reason).warn();
  if (callback) {
    callback(reason);
  }
}

void LiveKitRoomDelegate::markReconnecting(const std::string & reason)
{
  std::function<void(const std::string &)> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == livekit::ConnectionState::Reconnecting) {
      return;
    }
    state_ = livekit::ConnectionState::Reconnecting;
    callback = callbacks_.on_reconnecting;
  }

  LogEvent(kLogger, "room_reconnecting").field("reason", reason).warn();
  if (callback) {
    callback(reason);
  }
}

void LiveKitRoomDelegate::markReconnected()
{
  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != livekit::ConnectionState::Reconnecting) {
      state_ = livekit::ConnectionState::Connected;
      return;
    }
    state_ = livekit::ConnectionState::Connected;
    callback = callbacks_.on_reconnected;
  }

  LogEvent(kLogger, "room_reconnected").info();
  if (callback) {
    callback();
  }
}

void LiveKitRoomDelegate::onParticipantDisconnected(
  livekit::Room &, const livekit::ParticipantDisconnectedEvent & event)
{
  const auto * participant = event.participant;
  if (participant == nullptr) {
    return;
  }

  std::function<void(const livekit::ParticipantDisconnectedEvent &)> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != livekit::ConnectionState::Connected) {
      return;
    }
    callback = callbacks_.on_participant_disconnected;
  }

  if (!callback) {
    return;
  }

  if (participant->identity().empty()) {
    return;
  }

  callback(event);
}

void LiveKitRoomDelegate::onUserPacketReceived(livekit::Room &, const livekit::UserDataPacketEvent & event)
{
  std::function<void(const livekit::UserDataPacketEvent &)> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = callbacks_.on_user_packet_received;
  }

  if (callback) {
    callback(event);
  }
}

void LiveKitRoomDelegate::onConnectionStateChanged(livekit::Room &, const livekit::ConnectionStateChangedEvent & event)
{
  switch (event.state) {
    case livekit::ConnectionState::Connected:
      markReconnected();
      return;
    case livekit::ConnectionState::Reconnecting:
      markReconnecting("connection_state_reconnecting");
      return;
    default:
      requestReconnect("connection_state_disconnected");
      return;
  }
}

void LiveKitRoomDelegate::onDisconnected(livekit::Room &, const livekit::DisconnectedEvent &)
{
  requestReconnect("room_disconnected");
}

void LiveKitRoomDelegate::onReconnecting(livekit::Room &, const livekit::ReconnectingEvent &)
{
  markReconnecting("room_reconnecting");
}

void LiveKitRoomDelegate::onReconnected(livekit::Room &, const livekit::ReconnectedEvent &)
{
  markReconnected();
}

void LiveKitRoomDelegate::onRoomEos(livekit::Room &, const livekit::RoomEosEvent &)
{
  requestReconnect("room_eos");
}

}  // namespace livekit_ros2_bridge
