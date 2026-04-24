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
#include <utility>
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
  // Called when the SDK room connection state changes. The initial successful Connect() result is
  // also reported as Connected because it is established before normal delegate events are reliable.
  std::function<void(livekit::ConnectionState)> on_connection_state_changed;

  // Delivers one incoming LiveKit user packet on a connection-managed background thread; callbacks must
  // hand off ROS work instead of assuming executor-thread affinity.
  std::function<void(const livekit::UserDataPacketEvent &)> on_user_packet_received;

  // Called when a remote participant disconnects while the room is connected. During SDK reconnect,
  // the connection suppresses transient participant disconnects so leases can survive browser refreshes.
  // LiveKit owns the event lifetime, so callbacks must copy fields they keep beyond the call.
  std::function<void(const livekit::ParticipantDisconnectedEvent &)> on_participant_disconnected;
};

// Thread-safe transport facade around one SDK-owned room connection. Implementations may invoke
// callbacks from connection-managed threads.
class RoomConnection
{
public:
  virtual ~RoomConnection() = default;

  // Starts a one-shot background connection task using the supplied immutable LiveKit startup config.
  // Repeated calls after a successful start are ignored until stop() returns.
  virtual void start(LiveKitConfig config, RoomEventCallbacks callbacks) = 0;

  // Stops the active room and waits for any in-flight connection task to exit.
  virtual void stop() = 0;

  // Registers or replaces an RPC handler when a local participant is available.
  virtual bool registerRpc(const std::string & method, livekit::LocalParticipant::RpcHandler handler) = 0;
  virtual bool unregisterRpc(const std::string & method) = 0;

  // These publication calls require an active local participant. Implementations may throw if
  // used while disconnected, except tryPushDataTrack(), which reports expected push failures
  // in-band.
  virtual void publishData(
    const std::vector<std::uint8_t> & payload,
    bool reliable = true,
    const std::vector<std::string> & destination_identities = {},
    const std::string & topic = {}) = 0;

  virtual std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) = 0;

  virtual livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, const livekit::DataTrackFrame & frame) = 0;

  virtual void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;

  // Returned video tracks carry the SDK publication identity. Unpublishing a stale track after the
  // active SDK room is gone is a no-op.
  virtual std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & options) = 0;

  virtual void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) = 0;
};

std::unique_ptr<RoomConnection> createRoomConnection();

}  // namespace livekit_ros2_bridge
