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

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/remote_participant.h"
#include "livekit/room.h"
#include "livekit_room_delegate.hpp"

namespace livekit_ros2_bridge::test_support
{

struct LiveKitRoomDelegateTestEvents
{
  static livekit::RemoteParticipant makeRemoteParticipant(std::string identity)
  {
    return livekit::RemoteParticipant(
      livekit::FfiHandle{},
      "fake-participant-sid",
      "fake-participant-name",
      std::move(identity),
      "",
      std::unordered_map<std::string, std::string>{},
      livekit::ParticipantKind::Standard,
      livekit::DisconnectReason::Unknown);
  }

  static void emitReconnectRequested(LiveKitRoomDelegate & delegate, const std::string & reason)
  {
    livekit::Room room;
    if (reason == "room_eos") {
      livekit::RoomEosEvent event;
      delegate.onRoomEos(room, event);
      return;
    }
    if (reason == "connection_state_disconnected") {
      livekit::ConnectionStateChangedEvent event;
      event.state = livekit::ConnectionState::Disconnected;
      delegate.onConnectionStateChanged(room, event);
      return;
    }

    livekit::DisconnectedEvent event;
    delegate.onDisconnected(room, event);
  }

  static void emitReconnecting(LiveKitRoomDelegate & delegate, const std::string & reason)
  {
    livekit::Room room;
    if (reason == "connection_state_reconnecting") {
      livekit::ConnectionStateChangedEvent event;
      event.state = livekit::ConnectionState::Reconnecting;
      delegate.onConnectionStateChanged(room, event);
      return;
    }

    livekit::ReconnectingEvent event;
    delegate.onReconnecting(room, event);
  }

  static void emitReconnected(LiveKitRoomDelegate & delegate)
  {
    livekit::Room room;
    livekit::ReconnectedEvent event;
    delegate.onReconnected(room, event);
  }

  static void emitConnectionStateChanged(LiveKitRoomDelegate & delegate, livekit::ConnectionState state)
  {
    livekit::Room room;
    livekit::ConnectionStateChangedEvent event;
    event.state = state;
    delegate.onConnectionStateChanged(room, event);
  }

  static void emitParticipantDisconnected(LiveKitRoomDelegate & delegate, const std::string & identity)
  {
    livekit::ParticipantDisconnectedEvent event;
    if (identity.empty()) {
      livekit::Room room;
      delegate.onParticipantDisconnected(room, event);
      return;
    }

    auto participant = makeRemoteParticipant(identity);
    event.participant = &participant;
    livekit::Room room;
    delegate.onParticipantDisconnected(room, event);
  }

  static void emitUserPacket(
    LiveKitRoomDelegate & delegate, std::vector<std::uint8_t> payload, std::string topic, const std::string & identity)
  {
    livekit::UserDataPacketEvent event;
    event.data = std::move(payload);
    event.topic = std::move(topic);

    if (identity.empty()) {
      livekit::Room room;
      delegate.onUserPacketReceived(room, event);
      return;
    }

    auto participant = makeRemoteParticipant(identity);
    event.participant = &participant;
    livekit::Room room;
    delegate.onUserPacketReceived(room, event);
  }

  static void emitUserPacket(
    LiveKitRoomDelegate & delegate, const std::string & payload, std::string topic, const std::string & identity)
  {
    emitUserPacket(delegate, std::vector<std::uint8_t>(payload.begin(), payload.end()), std::move(topic), identity);
  }
};

}  // namespace livekit_ros2_bridge::test_support
