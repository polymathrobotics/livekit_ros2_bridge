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

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "livekit_room_delegate.hpp"
#include "livekit_room_delegate_test_support.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::LiveKitRoomDelegateTestEvents;

struct RecordingControl
{
  LiveKitRoomDelegate::ReconnectRequestHandler callback()
  {
    return [this](const std::string & reason) {
      if (reconnect_requested) {
        return false;
      }
      reconnect_requested = true;
      reconnect_reasons.push_back(reason);
      return true;
    };
  }

  bool reconnect_requested = false;
  std::vector<std::string> reconnect_reasons;
};

TEST(LiveKitRoomDelegateTest, ForwardsUserPackets)
{
  RoomEventCallbacks callbacks;
  bool packet_received = false;
  std::string topic;
  std::string identity;
  std::vector<std::uint8_t> payload;
  callbacks.on_user_packet_received = [&](const livekit::UserDataPacketEvent & event) {
    packet_received = true;
    topic = event.topic;
    identity = event.participant == nullptr ? "" : event.participant->identity();
    payload = event.data;
  };

  LiveKitRoomDelegate delegate(std::move(callbacks));
  LiveKitRoomDelegateTestEvents::emitUserPacket(
    delegate, std::vector<std::uint8_t>{1U, 2U, 3U}, "ros.test", "participant-1");

  EXPECT_TRUE(packet_received);
  EXPECT_EQ(topic, "ros.test");
  EXPECT_EQ(identity, "participant-1");
  EXPECT_EQ(payload, (std::vector<std::uint8_t>{1U, 2U, 3U}));
}

TEST(LiveKitRoomDelegateTest, SuppressesParticipantDisconnectsOutsideConnectedRoom)
{
  RoomEventCallbacks callbacks;
  std::vector<std::string> disconnected_identities;
  callbacks.on_participant_disconnected = [&](const std::string & identity) {
    disconnected_identities.push_back(identity);
  };

  RecordingControl control;
  LiveKitRoomDelegate delegate(std::move(callbacks));
  delegate.setReconnectRequestHandler(control.callback());

  LiveKitRoomDelegateTestEvents::emitParticipantDisconnected(delegate, "participant-1");
  EXPECT_TRUE(disconnected_identities.empty());

  delegate.roomConnected();
  LiveKitRoomDelegateTestEvents::emitParticipantDisconnected(delegate, "participant-1");
  EXPECT_EQ(disconnected_identities, (std::vector<std::string>{"participant-1"}));

  LiveKitRoomDelegateTestEvents::emitReconnecting(delegate, "room_reconnecting");
  LiveKitRoomDelegateTestEvents::emitParticipantDisconnected(delegate, "participant-2");
  EXPECT_EQ(disconnected_identities, (std::vector<std::string>{"participant-1"}));

  LiveKitRoomDelegateTestEvents::emitReconnected(delegate);
  LiveKitRoomDelegateTestEvents::emitParticipantDisconnected(delegate, "");
  LiveKitRoomDelegateTestEvents::emitParticipantDisconnected(delegate, "participant-3");
  EXPECT_EQ(disconnected_identities, (std::vector<std::string>{"participant-1", "participant-3"}));
}

TEST(LiveKitRoomDelegateTest, CoalescesReconnectNotifications)
{
  RoomEventCallbacks callbacks;
  std::vector<std::string> reconnect_reasons;
  std::vector<std::string> reconnecting_reasons;
  int reconnected_count = 0;
  callbacks.on_reconnect_requested = [&](const std::string & reason) { reconnect_reasons.push_back(reason); };
  callbacks.on_reconnecting = [&](const std::string & reason) { reconnecting_reasons.push_back(reason); };
  callbacks.on_reconnected = [&]() { ++reconnected_count; };

  RecordingControl control;
  LiveKitRoomDelegate delegate(std::move(callbacks));
  delegate.setReconnectRequestHandler(control.callback());

  LiveKitRoomDelegateTestEvents::emitReconnectRequested(delegate, "room_disconnected");
  LiveKitRoomDelegateTestEvents::emitReconnectRequested(delegate, "room_eos");
  LiveKitRoomDelegateTestEvents::emitConnectionStateChanged(delegate, livekit::ConnectionState::Reconnecting);
  LiveKitRoomDelegateTestEvents::emitReconnecting(delegate, "room_reconnecting");
  LiveKitRoomDelegateTestEvents::emitConnectionStateChanged(delegate, livekit::ConnectionState::Connected);
  LiveKitRoomDelegateTestEvents::emitReconnected(delegate);

  EXPECT_EQ(reconnect_reasons, (std::vector<std::string>{"room_disconnected"}));
  EXPECT_EQ(reconnecting_reasons, (std::vector<std::string>{"connection_state_reconnecting"}));
  EXPECT_EQ(reconnected_count, 1);
  EXPECT_EQ(control.reconnect_reasons, (std::vector<std::string>{"room_disconnected"}));
}

TEST(LiveKitRoomDelegateTest, ForwardsConnectedAndConnectionResetCallbacks)
{
  RoomEventCallbacks callbacks;
  int connected_count = 0;
  int reset_count = 0;
  callbacks.on_connected = [&]() { ++connected_count; };
  callbacks.on_connection_reset = [&]() { ++reset_count; };

  LiveKitRoomDelegate delegate(std::move(callbacks));

  delegate.roomConnected();
  delegate.roomConnectionReset();

  EXPECT_EQ(connected_count, 1);
  EXPECT_EQ(reset_count, 1);
}

}  // namespace
}  // namespace livekit_ros2_bridge
