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
