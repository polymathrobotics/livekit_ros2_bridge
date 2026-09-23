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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "core/room_connection.hpp"
#include "gtest/gtest.h"
#include "protocol/constants.hpp"

namespace livekit_ros2_bridge
{
namespace
{

// The detached-thread body of sendByteStream (construct/write/close against the LiveKit SDK) needs a
// live room and is exercised by integration, like the rest of SdkRoomConnection. These tests cover
// the synchronous guards that run before any thread is spawned, on a connection that was never
// started: payload validation and the local-participant precondition. The latter is the exact
// synchronous throw SubscriptionLeaseManager::dispatchEchoOnce catches to report "none".

std::shared_ptr<const std::vector<std::uint8_t>> makeCdr()
{
  return std::make_shared<const std::vector<std::uint8_t>>(std::vector<std::uint8_t>{0x01, 0x02, 0x03});
}

TEST(RoomConnectionSendByteStreamTest, NullPayloadThrowsInvalidArgument)
{
  const auto connection = createRoomConnection();
  EXPECT_THROW(
    connection->sendByteStream(protocol::kEchoOnceTopic, "/map", protocol::kCdrContentType, nullptr, "participant-1"),
    std::invalid_argument);
}

TEST(RoomConnectionSendByteStreamTest, UnavailableLocalParticipantThrowsRuntimeError)
{
  // Never started, so there is no local participant; the send must throw synchronously rather than
  // spawn a sender thread.
  const auto connection = createRoomConnection();
  EXPECT_THROW(
    connection->sendByteStream(protocol::kEchoOnceTopic, "/map", protocol::kCdrContentType, makeCdr(), "participant-1"),
    std::runtime_error);
}

}  // namespace
}  // namespace livekit_ros2_bridge
