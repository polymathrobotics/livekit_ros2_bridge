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
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "livekit/data_track_frame.h"
#include "livekit/local_participant.h"
#include "livekit/room_event_types.h"

namespace livekit
{
class LocalDataTrack;
class LocalVideoTrack;
struct LocalDataTrackTryPushError;
template <typename T, typename E>
class Result;
struct ParticipantDisconnectedEvent;
struct UserDataPacketEvent;
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

struct LiveKitConfig
{
  std::string url;
  std::string access_token;
};

struct RoomEventCallbacks
{
  // Initial successful Connect() is reported as Connected before delegate events are reliable.
  std::function<void(livekit::ConnectionState)> on_state_changed;

  // Runs on a connection-managed thread, not necessarily a ROS executor thread.
  std::function<void(const livekit::UserDataPacketEvent &)> on_user_packet_received;

  // SDK reconnect suppresses transient disconnects; LiveKit owns the event lifetime.
  std::function<void(const livekit::ParticipantDisconnectedEvent &)> on_participant_disconnected;
};

// Thread-safe facade around one SDK-owned room; callbacks may run on connection-managed threads.
class RoomConnection
{
public:
  virtual ~RoomConnection() = default;

  // Starts a background connection task. Repeated calls are ignored until stop() returns.
  virtual void start(LiveKitConfig config, RoomEventCallbacks callbacks) = 0;

  // Stops the active room and waits for the connection task to exit.
  virtual void stop() = 0;

  // A false return means active SDK registration failed; the handler remains saved for the next connection.
  virtual bool registerRpc(const std::string & method, livekit::LocalParticipant::RpcHandler handler) = 0;

  virtual bool unregisterRpc(const std::string & method) = 0;

  // Publish calls require an active local participant and may throw while disconnected.
  virtual void publishData(
    const std::vector<std::uint8_t> & payload,
    bool reliable = true,
    const std::vector<std::string> & destination_identities = {},
    const std::string & topic = {}) = 0;

  virtual std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) = 0;

  virtual livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, const livekit::DataTrackFrame & frame) = 0;

  virtual void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;

  // Returned video tracks carry SDK publication identity; stale-track unpublishes are no-ops.
  virtual std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & options) = 0;

  virtual void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) = 0;
};

std::unique_ptr<RoomConnection> createRoomConnection();

}  // namespace livekit_ros2_bridge
