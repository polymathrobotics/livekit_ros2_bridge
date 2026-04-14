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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "video_stream_config.hpp"

namespace livekit
{
class LocalDataTrack;
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

struct LiveKitConfig
{
  std::string url;
  std::string room;
  std::string access_token;
};

struct RpcInvocation
{
  RpcInvocation() = default;

  RpcInvocation(std::string caller_identity, std::string payload)
  : caller_identity(std::move(caller_identity))
  , payload(std::move(payload))
  {}

  RpcInvocation(std::string caller_identity, std::string request_id, std::string payload)
  : caller_identity(std::move(caller_identity))
  , request_id(std::move(request_id))
  , payload(std::move(payload))
  {}

  std::string caller_identity;
  std::string request_id;
  std::string payload;
};

// Throw from an RpcHandler to propagate an explicit RPC error code/message back to the remote
// caller. Other exceptions are treated as generic internal failures.
class RpcHandlerError : public std::runtime_error
{
public:
  RpcHandlerError(std::uint32_t code, std::string message)
  : std::runtime_error(std::move(message))
  , code_(code)
  {}

  std::uint32_t code() const noexcept
  {
    return code_;
  }

private:
  std::uint32_t code_;
};

using RpcHandler = std::function<std::optional<std::string>(const RpcInvocation &)>;

struct IncomingPacket
{
  std::vector<std::uint8_t> payload;
  std::string topic;
  std::string requester_identity;
};

struct OutgoingPacket
{
  std::vector<std::uint8_t> payload;
  std::vector<std::string> recipient_identities;
  std::string topic;
};

struct VideoTrackHandle
{
  // Opaque handle used only to request a later unpublish. Implementations may invalidate it after
  // unpublish or any connection reset/reconnect.
  std::string name;
};

enum class DataTrackPushErrorCode
{
  kUnknown,
  kInvalidHandle,
  kTrackUnpublished,
  kQueueFull,
  kInternal,
};

struct DataTrackPushError
{
  DataTrackPushErrorCode code = DataTrackPushErrorCode::kUnknown;
  std::string message;
};

class DataTrackPushResult
{
public:
  // Non-throwing result for the hot data-track push path, where stale handles and queue pressure
  // are expected runtime conditions.
  static DataTrackPushResult success();
  static DataTrackPushResult failure(DataTrackPushError error);

  bool ok() const noexcept;
  bool hasError() const noexcept;
  explicit operator bool() const noexcept;
  // Throws std::logic_error when no error is present.
  const DataTrackPushError & error() const;

private:
  explicit DataTrackPushResult(std::optional<DataTrackPushError> error);

  std::optional<DataTrackPushError> error_;
};

struct RoomEventCallbacks
{
  std::function<void()> on_connected;

  // Delivers one incoming packet on a connection-managed background thread; callbacks must
  // hand off ROS work instead of assuming executor-thread affinity.
  std::function<void(const IncomingPacket &)> on_incoming_packet_received;

  // Called when a remote participant disconnects outside reconnect handling. During reconnect, the
  // connection suppresses transient participant disconnects so leases can survive browser refreshes.
  std::function<void(const std::string &)> on_remote_participant_disconnected;

  // Called once when the current room connection begins a reconnect episode. The reason is a
  // stable internal string such as `room_disconnected` or `connection_state_disconnected`.
  std::function<void(const std::string &)> on_reconnect_requested;

  // Called after a connected room connection has been torn down and any per-connection state
  // should be rebuilt on the next connect. Final stop() does not fire this callback.
  std::function<void()> on_connection_reset;
};

// Thread-safe transport facade around a reconnecting room connection. Implementations own background
// worker state and may invoke callbacks from connection-managed threads.
class RoomConnection
{
public:
  virtual ~RoomConnection() = default;

  // Starts the background connection and reconnect loop using the supplied immutable LiveKit
  // startup config. Repeated calls after a successful start are ignored until stop() returns.
  virtual void start(LiveKitConfig config, RoomEventCallbacks callbacks) = 0;

  // Stops the reconnect loop and waits for any connection-owned background thread to exit.
  virtual void stop() = 0;

  // Registers or replaces an RPC handler and reapplies it after reconnects when a local
  // participant is available.
  virtual bool registerRpc(const std::string & method, RpcHandler handler) = 0;
  virtual bool unregisterRpc(const std::string & method) = 0;

  // These publication calls require an active local participant. Implementations may throw if
  // used while disconnected, except tryPushDataTrack(), which reports expected push failures
  // in-band.
  virtual void publishPacket(const OutgoingPacket & packet) = 0;
  virtual std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) = 0;
  virtual DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, std::vector<std::uint8_t> payload) = 0;
  virtual void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;

  virtual std::shared_ptr<VideoTrackHandle> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const VideoPublishConfig & config) = 0;
  // Best-effort no-op for null, already-unpublished, or reconnect-stale handles.
  virtual void unpublishVideoTrack(const std::shared_ptr<VideoTrackHandle> & handle) = 0;
};

std::unique_ptr<RoomConnection> createRoomConnection();

}  // namespace livekit_ros2_bridge
