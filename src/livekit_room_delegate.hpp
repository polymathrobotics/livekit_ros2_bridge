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

#include <functional>
#include <mutex>
#include <string>

#include "livekit/room_delegate.h"
#include "livekit/room_event_types.h"
#include "room_connection.hpp"

namespace livekit_ros2_bridge
{

// Runtime-owned adapter from LiveKit SDK room events into the bridge's stable room callbacks.
//
// livekit::Room stores a non-owning RoomDelegate pointer. Runtime therefore owns this delegate and
// RoomConnection only borrows it while its worker and SDK room are alive.
class LiveKitRoomDelegate final : public livekit::RoomDelegate
{
public:
  using ReconnectRequestHandler = std::function<bool(const std::string &)>;

  explicit LiveKitRoomDelegate(RoomEventCallbacks callbacks);

  void setReconnectRequestHandler(ReconnectRequestHandler handler);
  void clearReconnectRequestHandler();

  void roomConnected();
  void roomConnectionReset();

  void onParticipantDisconnected(livekit::Room & room, const livekit::ParticipantDisconnectedEvent & event) override;
  void onUserPacketReceived(livekit::Room & room, const livekit::UserDataPacketEvent & event) override;
  void onConnectionStateChanged(livekit::Room & room, const livekit::ConnectionStateChangedEvent & event) override;
  void onDisconnected(livekit::Room & room, const livekit::DisconnectedEvent & event) override;
  void onReconnecting(livekit::Room & room, const livekit::ReconnectingEvent & event) override;
  void onReconnected(livekit::Room & room, const livekit::ReconnectedEvent & event) override;
  void onRoomEos(livekit::Room & room, const livekit::RoomEosEvent & event) override;

private:
  void requestReconnect(const std::string & reason);
  void markReconnecting(const std::string & reason);
  void markReconnected();

  std::mutex mutex_;
  RoomEventCallbacks callbacks_;
  ReconnectRequestHandler request_reconnect_;
  livekit::ConnectionState state_ = livekit::ConnectionState::Disconnected;
};

}  // namespace livekit_ros2_bridge
