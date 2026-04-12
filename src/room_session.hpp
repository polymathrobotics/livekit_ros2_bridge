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

#include "video_config.hpp"

namespace livekit
{
class LocalDataTrack;
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

struct RoomConnectionConfig
{
  std::string url;
  std::string room;
};

struct RpcInvocation
{
  RpcInvocation() = default;

  // Preserve the historical two-argument construction shape used by tests and
  // internal call sites that only provide caller identity plus payload.
  RpcInvocation(std::string caller_identity_in, std::string payload_in)
  : caller_identity(std::move(caller_identity_in))
  , payload(std::move(payload_in))
  {}

  RpcInvocation(std::string caller_identity_in, std::string request_id_in, std::string payload_in)
  : caller_identity(std::move(caller_identity_in))
  , request_id(std::move(request_id_in))
  , payload(std::move(payload_in))
  {}

  std::string caller_identity;
  std::string request_id;
  std::string payload;
};

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

struct IncomingControlPacket
{
  std::vector<std::uint8_t> payload;
  std::string control_topic;
  std::string requester_identity;
};

struct OutgoingControlPacket
{
  std::vector<std::uint8_t> payload;
  std::vector<std::string> recipient_identities;
  std::string control_topic;
};

struct PublishedVideoTrack
{
  std::string track_name;
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
  static DataTrackPushResult success();
  static DataTrackPushResult failure(DataTrackPushError error);

  bool ok() const noexcept;
  bool hasError() const noexcept;
  explicit operator bool() const noexcept;
  const DataTrackPushError & error() const;

private:
  explicit DataTrackPushResult(std::optional<DataTrackPushError> error);

  std::optional<DataTrackPushError> error_;
};

struct RoomSessionCallbacks
{
  // Called when a room connection becomes active.
  std::function<void()> on_connected;
  // Called when the current room connection begins a reconnect episode. The reason is a stable
  // internal string such as `room_disconnected` or `connection_state_disconnected`.
  std::function<void(const std::string &)> on_reconnect_requested;
  // Called after a connected room session has been torn down and any per-session room state
  // should be rebuilt on the next connect.
  std::function<void()> on_session_reset;
  // Called when a requester identity disconnects outside reconnect handling. During reconnect, the
  // session suppresses transient participant disconnects so leases can survive browser refreshes.
  std::function<void(const std::string &)> on_participant_disconnected;
  // Delivers one incoming control packet. Callbacks may run on session-managed background threads
  // and must hand off ROS work instead of assuming executor-thread affinity.
  std::function<void(const IncomingControlPacket &)> on_incoming_control_packet_received;
};

class RoomSession
{
public:
  virtual ~RoomSession() = default;

  // Starts the background connection and reconnect loop. Repeated calls after a successful start
  // are ignored until stop() returns.
  virtual void start(
    RoomConnectionConfig config,
    std::string access_token,
    RoomSessionCallbacks callbacks,
    std::chrono::milliseconds initial_backoff,
    std::chrono::milliseconds max_backoff) = 0;
  // Stops the reconnect loop and waits for any session-owned background thread to exit.
  virtual void stop() = 0;
  // Registers or replaces an RPC handler and reapplies it after reconnects when a local
  // participant is available.
  virtual bool registerRpcMethod(const std::string & method_name, RpcHandler handler) = 0;
  virtual bool unregisterRpcMethod(const std::string & method_name) = 0;
  virtual void publishControlPacket(const OutgoingControlPacket & packet) = 0;
  virtual std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) = 0;
  virtual DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, std::vector<std::uint8_t> payload) = 0;
  virtual void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;
  virtual std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const VideoPublishConfig & publish_config) = 0;
  virtual void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) = 0;
};

std::unique_ptr<RoomSession> makeRoomSession();

}  // namespace livekit_ros2_bridge
