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
  std::string identity;
};

struct AccessToken
{
  std::string value;
  // Best-effort minting metadata used only for reconnect and refresh timing.
  std::optional<std::chrono::system_clock::time_point> issued_at;
  std::optional<std::chrono::system_clock::time_point> expires_at;
  // True when getToken() can mint a replacement token for the same bridge identity.
  bool refreshable = false;
};

class AccessTokenSource
{
public:
  virtual ~AccessTokenSource() = default;
  // Returns the token the bridge should use for the next room connection attempt. The config
  // carries the bridge's own LiveKit identity, not a remote requester or RPC caller identity.
  virtual AccessToken getToken(const RoomConnectionConfig & config) = 0;
};

class StaticTokenSource final : public AccessTokenSource
{
public:
  explicit StaticTokenSource(std::string token);
  AccessToken getToken(const RoomConnectionConfig & config) override;

private:
  std::string token_;
};

class ApiKeyAccessTokenSource final : public AccessTokenSource
{
public:
  ApiKeyAccessTokenSource(std::string api_key, std::string api_secret, std::chrono::seconds ttl);

  AccessToken getToken(const RoomConnectionConfig & config) override;

private:
  std::string api_key_;
  std::string api_secret_;
  std::chrono::seconds ttl_;
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

struct RoomSessionCallbacks
{
  // Called after a connected session has been torn down and any per-connection room state should
  // be rebuilt on the next connect.
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
    std::shared_ptr<AccessTokenSource> access_token_source,
    RoomSessionCallbacks callbacks,
    std::chrono::milliseconds initial_backoff,
    std::chrono::milliseconds max_backoff,
    std::chrono::seconds refresh_margin) = 0;
  // Stops the reconnect loop and waits for any session-owned background thread to exit.
  virtual void stop() = 0;
  // Registers or replaces an RPC handler and reapplies it after reconnects when a local
  // participant is available.
  virtual bool registerRpcMethod(const std::string & method_name, RpcHandler handler) = 0;
  virtual bool unregisterRpcMethod(const std::string & method_name) = 0;
  virtual void publishControlPacket(const OutgoingControlPacket & packet) = 0;
  virtual std::shared_ptr<livekit::LocalDataTrack> publishCdrTrack(const std::string & name) = 0;
  virtual void unpublishCdrTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;
  virtual std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const VideoPublishConfig & publish_config) = 0;
  virtual void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) = 0;
};

std::unique_ptr<RoomSession> makeRoomSession();

}  // namespace livekit_ros2_bridge
