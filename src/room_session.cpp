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

#include "room_session.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/livekit.h"
#include "livekit/local_data_track.h"
#include "livekit/local_video_track.h"
#include "livekit/remote_participant.h"
#include "livekit/remote_track_publication.h"
#include "livekit/room_delegate.h"
#include "livekit/rpc_error.h"
#include "livekit/track.h"
#include "livekit/video_source.h"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "utils/livekit_access_token.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using Clock = std::chrono::system_clock;
const auto kRoomSessionLogger = rclcpp::get_logger("livekit_ros2_bridge.room_session");
constexpr char kUnknownLogValue[] = "<unknown>";
constexpr char kUnsetLogValue[] = "<unset>";
constexpr auto kRefreshStatePollInterval = std::chrono::seconds(1);

const char * requestIdForLog(const livekit::RpcInvocationData & invocation)
{
  return invocation.request_id.empty() ? kUnknownLogValue : invocation.request_id.c_str();
}

const char * requesterIdentityForLog(const livekit::RpcInvocationData & invocation)
{
  return invocation.caller_identity.empty() ? kUnknownLogValue : invocation.caller_identity.c_str();
}

livekit::LocalParticipant::RpcHandler makeLiveKitRpcHandler(const std::string & method_name, const RpcHandler & handler)
{
  return [method_name, handler](const livekit::RpcInvocationData & invocation) -> std::optional<std::string> {
    const char * requester_identity = requesterIdentityForLog(invocation);
    const char * request_id = requestIdForLog(invocation);
    LogEvent(kRoomSessionLogger, "rpc_request_received")
      .kv("method", method_name)
      .kv("request_id", request_id)
      .kv("requester_identity", requester_identity)
      .kv("payload_bytes", invocation.payload.size())
      .info();

    try {
      return handler(
        RpcInvocation{
          invocation.caller_identity,
          invocation.request_id,
          invocation.payload,
        });
    } catch (const RpcHandlerError & exc) {
      throw livekit::RpcError(exc.code(), exc.what());
    } catch (const std::exception & exc) {
      LogEvent(kRoomSessionLogger, "rpc_request_failed")
        .kv("reason", "internal")
        .kv("method", method_name)
        .kv("request_id", request_id)
        .kv("requester_identity", requester_identity)
        .kv("error", exc.what())
        .error();
      throw livekit::RpcError(protocol::kRpcErrorInternal, "Internal error handling RPC method");
    } catch (...) {
      LogEvent(kRoomSessionLogger, "rpc_request_failed")
        .kv("reason", "internal")
        .kv("method", method_name)
        .kv("request_id", request_id)
        .kv("requester_identity", requester_identity)
        .kv("error", "unknown_exception")
        .error();
      throw livekit::RpcError(protocol::kRpcErrorInternal, "Internal error handling RPC method");
    }
  };
}

std::optional<Clock::time_point> computeRefreshDeadline(const AccessToken & token, std::chrono::seconds refresh_margin)
{
  if (!token.expires_at.has_value()) {
    return std::nullopt;
  }

  auto effective_margin = std::max(refresh_margin, std::chrono::seconds(0));
  if (token.issued_at.has_value() && *token.expires_at > *token.issued_at) {
    const auto ttl = std::chrono::duration_cast<std::chrono::seconds>(*token.expires_at - *token.issued_at);
    const auto half_ttl = ttl / 2;
    effective_margin = std::min(effective_margin, half_ttl);
  }

  return *token.expires_at - effective_margin;
}

std::string formatTokenExpiry(const std::optional<Clock::time_point> & expires_at)
{
  if (!expires_at.has_value()) {
    return kUnknownLogValue;
  }

  const auto unix_seconds = std::chrono::duration_cast<std::chrono::seconds>(expires_at->time_since_epoch()).count();
  return std::to_string(unix_seconds);
}

class LiveKitRoomSession final : public RoomSession, public livekit::RoomDelegate
{
public:
  LiveKitRoomSession() = default;

  ~LiveKitRoomSession() override
  {
    stop();
  }

  void start(
    RoomConnectionConfig config,
    std::shared_ptr<AccessTokenSource> access_token_source,
    RoomSessionCallbacks callbacks,
    std::chrono::milliseconds initial_backoff,
    std::chrono::milliseconds max_backoff,
    std::chrono::seconds refresh_margin) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (thread_started_) {
      // The bridge owns a single reconnect loop per session instance.
      return;
    }

    if (access_token_source == nullptr) {
      throw std::invalid_argument("LiveKit access_token_source is required.");
    }

    config_ = std::move(config);
    token_source_ = std::move(access_token_source);
    callbacks_ = std::move(callbacks);
    initial_backoff_ = std::max(initial_backoff, std::chrono::milliseconds(0));
    max_backoff_ = std::max(max_backoff, initial_backoff_);
    refresh_margin_ = std::max(refresh_margin, std::chrono::seconds(0));
    stop_requested_ = false;
    reconnect_requested_ = false;
    static_expiry_warned_ = false;
    last_reconnect_reason_.clear();
    thread_started_ = true;
    LogEvent(kRoomSessionLogger, "room_session_start_requested")
      .kv("phase", "startup")
      .kvOr("room", config_.room, kUnsetLogValue)
      .kvOr("identity", config_.identity, kUnsetLogValue)
      .info();
    worker_thread_ = std::thread([this]() { run(); });
  }

  void stop() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!thread_started_) {
      return;
    }
    stop_requested_ = true;
    reconnect_requested_ = true;
    condition_.notify_all();
    lock.unlock();

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    lock.lock();
    thread_started_ = false;
  }

  bool registerRpcMethod(const std::string & method_name, RpcHandler handler) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpc_handlers_[method_name] = std::move(handler);
    return registerRpcMethodLocked(method_name);
  }

  bool unregisterRpcMethod(const std::string & method_name) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpc_handlers_.erase(method_name);

    auto * participant = localParticipantLocked();
    if (participant == nullptr) {
      return true;
    }

    try {
      participant->unregisterRpcMethod(method_name);
    } catch (const std::exception & exc) {
      LogEvent(kRoomSessionLogger, "rpc_method_unregistration_failed")
        .kv("method", method_name)
        .kv("error", exc.what())
        .error();
      return false;
    }
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket & packet) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * participant = localParticipantLocked();
    if (participant == nullptr) {
      throw std::runtime_error("LiveKit local participant unavailable.");
    }

    participant->publishData(packet.payload, true, packet.recipient_identities, packet.control_topic);
  }

  std::shared_ptr<livekit::LocalDataTrack> publishCdrTrack(const std::string & name) override
  {
    std::shared_ptr<livekit::Room> room;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Keep the Room alive after releasing the mutex because publishCdrTrack()
      // may block in the SDK while reconnect/teardown clears room_ on another thread.
      // The tradeoff is that teardown can be delayed briefly, but we avoid
      // holding the session mutex across a potentially slow SDK call.
      room = room_;
    }

    if (room == nullptr) {
      throw std::runtime_error("LiveKit local participant unavailable.");
    }

    auto * participant = room->localParticipant();
    if (participant == nullptr) {
      throw std::runtime_error("LiveKit local participant unavailable.");
    }

    auto result = participant->publishDataTrack(name);
    if (!result) {
      throw std::runtime_error("Failed to publish CDR track '" + name + "': " + result.error().message);
    }
    return result.value();
  }

  void unpublishCdrTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * participant = localParticipantLocked();
    if (participant == nullptr || track == nullptr) {
      return;
    }
    participant->unpublishDataTrack(track);
  }

  std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name, const std::shared_ptr<livekit::VideoSource> & source) override
  {
    if (track_name.empty()) {
      throw std::invalid_argument("Video track name is required.");
    }
    if (source == nullptr) {
      throw std::invalid_argument("Video source is required.");
    }

    std::shared_ptr<livekit::Room> room;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      room = room_;
    }

    if (room == nullptr) {
      throw std::runtime_error("LiveKit local participant unavailable.");
    }

    auto * participant = room->localParticipant();
    if (participant == nullptr) {
      throw std::runtime_error("LiveKit local participant unavailable.");
    }

    auto published_track = std::make_shared<PublishedVideoTrack>();
    published_track->track_name = track_name;

    auto local_track = participant->publishVideoTrack(track_name, source, livekit::TrackSource::SOURCE_CAMERA);
    if (local_track == nullptr || local_track->publication() == nullptr) {
      throw std::runtime_error("Failed to publish video track '" + track_name + "'.");
    }
    LogEvent(kRoomSessionLogger, "video_track_published")
      .kv("track_name", track_name)
      .kv("track_sid", local_track->publication()->sid())
      .info();

    std::lock_guard<std::mutex> lock(mutex_);
    published_video_tracks_[published_track.get()] = std::move(local_track);
    return published_track;
  }

  void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) override
  {
    std::shared_ptr<livekit::LocalVideoTrack> local_track;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (track == nullptr) {
        return;
      }

      const auto it = published_video_tracks_.find(track.get());
      if (it == published_video_tracks_.end()) {
        return;
      }
      local_track = std::move(it->second);
      published_video_tracks_.erase(it);
    }

    if (local_track == nullptr) {
      return;
    }

    const auto publication = local_track->publication();
    if (publication == nullptr) {
      return;
    }
    LogEvent(kRoomSessionLogger, "video_track_unpublishing")
      .kv("track_name", track->track_name)
      .kv("track_sid", publication->sid())
      .info();

    std::lock_guard<std::mutex> lock(mutex_);
    auto * participant = localParticipantLocked();
    if (participant == nullptr) {
      return;
    }
    participant->unpublishTrack(publication->sid());
  }

  void onParticipantDisconnected(livekit::Room &, const livekit::ParticipantDisconnectedEvent & event) override
  {
    std::function<void(const std::string &)> callback;
    std::string identity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!participant_disconnects_enabled_) {
        return;
      }
      callback = callbacks_.on_participant_disconnected;
      if (event.participant != nullptr) {
        identity = event.participant->identity();
      }
    }

    if (!identity.empty() && callback) {
      callback(identity);
    }
  }

  void onUserPacketReceived(livekit::Room &, const livekit::UserDataPacketEvent & event) override
  {
    std::function<void(const IncomingControlPacket &)> callback;
    IncomingControlPacket packet;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = callbacks_.on_incoming_control_packet_received;
      if (!callback) {
        return;
      }

      packet.payload = event.data;
      packet.control_topic = event.topic;
      if (event.participant != nullptr) {
        packet.requester_identity = event.participant->identity();
      }
    }

    callback(packet);
  }

  void onConnectionStateChanged(livekit::Room &, const livekit::ConnectionStateChangedEvent & event) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // LiveKit can report participant disconnects as part of a transport loss. Suppress those
      // until the room is fully connected again so runtime state survives reconnects and refreshes.
      participant_disconnects_enabled_ = event.state == livekit::ConnectionState::Connected;
    }

    if (event.state == livekit::ConnectionState::Disconnected) {
      requestReconnect("connection_state_disconnected");
    }
  }

  void onDisconnected(livekit::Room &, const livekit::DisconnectedEvent &) override
  {
    resetAndReconnect("room_disconnected");
  }

  void onRoomEos(livekit::Room &, const livekit::RoomEosEvent &) override
  {
    resetAndReconnect("room_eos");
  }

private:
  void resetAndReconnect(const char * reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      participant_disconnects_enabled_ = false;
    }
    requestReconnect(reason);
  }

  void run()
  {
    if (!livekit::initialize()) {
      LogEvent(kRoomSessionLogger, "livekit_initialize_failed")
        .kv("phase", "startup")
        .kv("reason", "initialize_returned_false")
        .error();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      livekit_initialized_ = true;
    }
    LogEvent(kRoomSessionLogger, "livekit_initialized").kv("phase", "startup").info();

    auto backoff = initialBackoff();
    bool immediate_retry = false;
    while (!stopRequested()) {
      const bool connected = connectOnce();
      if (!connected) {
        immediate_retry = false;
      } else {
        backoff = initialBackoff();
        immediate_retry = waitForRefreshOrDisconnect();
      }

      const bool notify_reset = connected && !stopRequested();
      clearRoomState(notify_reset);

      if (stopRequested()) {
        break;
      }

      if (immediate_retry) {
        continue;
      }

      if (backoff.count() > 0) {
        LogEvent(kRoomSessionLogger, "room_reconnect_backoff")
          .kv("phase", "reconnect")
          .kv("reason", lastReconnectReason())
          .kvOr("room", config_.room, kUnsetLogValue)
          .kvOr("identity", config_.identity, kUnsetLogValue)
          .kv("delay_seconds", backoff.count() / 1000.0)
          .warn();
        waitForStop(backoff);
      }
      backoff = std::min(backoff * 2, maxBackoff());
    }

    clearRoomState(false);
    if (livekit_initialized_) {
      livekit::shutdown();
      LogEvent(kRoomSessionLogger, "livekit_shutdown_complete").kv("phase", "shutdown").info();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    livekit_initialized_ = false;
  }

  bool connectOnce()
  {
    RoomConnectionConfig config;
    std::shared_ptr<AccessTokenSource> token_source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      config = config_;
      token_source = token_source_;
    }

    AccessToken token;
    try {
      token = token_source->getToken(config);
    } catch (const std::exception & exc) {
      LogEvent(kRoomSessionLogger, "room_token_load_failed")
        .kv("phase", "connect")
        .kv("reason", "exception")
        .kvOr("room", config.room, kUnsetLogValue)
        .kvOr("identity", config.identity, kUnsetLogValue)
        .kv("error", exc.what())
        .error();
      return false;
    }

    if (token.value.empty()) {
      LogEvent(kRoomSessionLogger, "room_token_load_failed")
        .kv("phase", "connect")
        .kv("reason", "empty_token")
        .kvOr("room", config.room, kUnsetLogValue)
        .kvOr("identity", config.identity, kUnsetLogValue)
        .error();
      return false;
    }

    logTokenState(token);

    auto room = std::make_shared<livekit::Room>();
    room->setDelegate(this);

    livekit::RoomOptions room_options;
    room_options.auto_subscribe = true;
    // Reuse the connected transport for publish+subscribe so data-track
    // publication does not depend on bringing up a second peer connection.
    room_options.single_peer_connection = true;

    LogEvent(kRoomSessionLogger, "room_connect_begin")
      .kv("phase", "connect")
      .kv("url", config.url)
      .kv("room", config.room)
      .kvOr("identity", config.identity, kUnsetLogValue)
      .info();

    try {
      if (!room->Connect(config.url, token.value, room_options)) {
        LogEvent(kRoomSessionLogger, "room_connect_failed")
          .kv("phase", "connect")
          .kv("reason", "connect_returned_false")
          .kv("url", config.url)
          .kv("room", config.room)
          .kvOr("identity", config.identity, kUnsetLogValue)
          .error();
        room->setDelegate(nullptr);
        return false;
      }
    } catch (const std::exception & exc) {
      LogEvent(kRoomSessionLogger, "room_connect_failed")
        .kv("phase", "connect")
        .kv("reason", "exception")
        .kv("url", config.url)
        .kv("room", config.room)
        .kvOr("identity", config.identity, kUnsetLogValue)
        .kv("error", exc.what())
        .error();
      room->setDelegate(nullptr);
      return false;
    } catch (...) {
      LogEvent(kRoomSessionLogger, "room_connect_failed")
        .kv("phase", "connect")
        .kv("reason", "unknown_exception")
        .kv("url", config.url)
        .kv("room", config.room)
        .kvOr("identity", config.identity, kUnsetLogValue)
        .error();
      room->setDelegate(nullptr);
      return false;
    }

    auto * participant = room->localParticipant();
    if (participant == nullptr) {
      LogEvent(kRoomSessionLogger, "room_connect_failed")
        .kv("phase", "connect")
        .kv("reason", "local_participant_unavailable")
        .kv("url", config.url)
        .kv("room", config.room)
        .kvOr("identity", config.identity, kUnsetLogValue)
        .error();
      room->setDelegate(nullptr);
      return false;
    }

    bool rpc_methods_registered = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      room_ = std::move(room);
      current_token_ = token;
      reconnect_requested_ = false;
      static_expiry_warned_ = false;
      participant_disconnects_enabled_ = true;
      rpc_methods_registered = registerAllRpcMethodsLocked();
    }

    if (!rpc_methods_registered) {
      LogEvent(kRoomSessionLogger, "rpc_registration_incomplete").kv("phase", "connect").error();
      clearRoomState(false);
      return false;
    }

    const livekit::RoomInfoData room_info = roomInfoSnapshot();
    const char * room_name = room_info.name.empty() ? kUnknownLogValue : room_info.name.c_str();
    const char * room_sid = room_info.sid.has_value() ? room_info.sid->c_str() : kUnknownLogValue;
    const std::string & local_identity = participant->identity();
    const char * identity = local_identity.empty() ? kUnknownLogValue : local_identity.c_str();
    LogEvent(kRoomSessionLogger, "room_connected")
      .kv("phase", "connect")
      .kv("room", room_name)
      .kv("sid", room_sid)
      .kv("identity", identity)
      .info();
    return true;
  }

  bool waitForRefreshOrDisconnect()
  {
    while (!stopRequested()) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (reconnect_requested_) {
          return false;
        }
        condition_.wait_for(lock, kRefreshStatePollInterval);
        if (reconnect_requested_) {
          return false;
        }
      }

      const auto now = Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if (!current_token_.expires_at.has_value()) {
        continue;
      }

      if (current_token_.refreshable) {
        // Refresh by reconnecting with a newly minted token before exp, rather than mutating the
        // active LiveKit connection in place.
        const auto deadline = computeRefreshDeadline(current_token_, refresh_margin_);
        if (deadline.has_value() && now >= *deadline) {
          LogEvent(kRoomSessionLogger, "room_reconnect_requested")
            .kv("phase", "runtime")
            .kv("reason", "token_refresh_due")
            .kvOr("room", config_.room, kUnsetLogValue)
            .kvOr("identity", config_.identity, kUnsetLogValue)
            .kv("exp_unix", formatTokenExpiry(current_token_.expires_at))
            .info();
          reconnect_requested_ = true;
          last_reconnect_reason_ = "token_refresh_due";
          return true;
        }
        continue;
      }

      const auto warn_deadline = *current_token_.expires_at - refresh_margin_;
      if (!static_expiry_warned_ && now >= warn_deadline) {
        static_expiry_warned_ = true;
        LogEvent(kRoomSessionLogger, "room_token_expiry_warning")
          .kv("phase", "runtime")
          .kv("mode", "static")
          .kv("reason", "automatic_refresh_unavailable")
          .kv("exp_unix", formatTokenExpiry(current_token_.expires_at))
          .warn();
      }
    }

    return false;
  }

  void clearRoomState(bool notify_session_reset)
  {
    std::shared_ptr<livekit::Room> room;
    std::function<void()> callback;
    std::string reconnect_reason;
    std::string room_name;
    std::string identity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      participant_disconnects_enabled_ = false;
      room = std::move(room_);
      published_video_tracks_.clear();
      current_token_ = AccessToken{};
      reconnect_requested_ = false;
      reconnect_reason = last_reconnect_reason_;
      room_name = config_.room;
      identity = config_.identity;
      last_reconnect_reason_.clear();
      callback = callbacks_.on_session_reset;
    }

    if (room != nullptr) {
      room->setDelegate(nullptr);
      room.reset();
    }

    // The runtime only rebuilds per-connection ROS state after an actual disconnect or reconnect
    // cycle, not during final stop().
    if (notify_session_reset && callback) {
      LogEvent(kRoomSessionLogger, "room_session_reset")
        .kv("phase", "reconnect")
        .kv("reason", reconnect_reason.empty() ? "session_reset" : reconnect_reason.c_str())
        .kvOr("room", room_name, kUnsetLogValue)
        .kvOr("identity", identity, kUnsetLogValue)
        .info();
      callback();
    }
  }

  void requestReconnect(const char * reason)
  {
    std::string room_name;
    std::string identity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (reconnect_requested_) {
        condition_.notify_all();
        return;
      }

      reconnect_requested_ = true;
      last_reconnect_reason_ = reason;
      room_name = config_.room;
      identity = config_.identity;
      condition_.notify_all();
    }

    LogEvent(kRoomSessionLogger, "room_reconnect_requested")
      .kv("phase", "runtime")
      .kv("reason", reason)
      .kvOr("room", room_name, kUnsetLogValue)
      .kvOr("identity", identity, kUnsetLogValue)
      .warn();
  }

  std::string lastReconnectReason() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_reconnect_reason_.empty() ? "retry_backoff" : last_reconnect_reason_;
  }

  bool registerAllRpcMethodsLocked()
  {
    bool all_registered = true;
    for (const auto & entry : rpc_handlers_) {
      if (!registerRpcMethodLocked(entry.first)) {
        all_registered = false;
      }
    }
    return all_registered;
  }

  bool registerRpcMethodLocked(const std::string & method_name)
  {
    auto * participant = localParticipantLocked();
    const auto handler_it = rpc_handlers_.find(method_name);
    if (participant == nullptr || handler_it == rpc_handlers_.end()) {
      return true;
    }

    try {
      participant->unregisterRpcMethod(method_name);
    } catch (const std::exception &) {
      // Ignore missing-method errors; we only need a clean re-register.
    }

    try {
      participant->registerRpcMethod(method_name, makeLiveKitRpcHandler(method_name, handler_it->second));
    } catch (const std::exception & exc) {
      LogEvent(kRoomSessionLogger, "rpc_method_registration_failed")
        .kv("method", method_name)
        .kv("error", exc.what())
        .error();
      return false;
    }
    return true;
  }

  livekit::LocalParticipant * localParticipantLocked() const
  {
    if (room_ == nullptr) {
      return nullptr;
    }
    return room_->localParticipant();
  }

  static bool hasActiveVideoTrack(const livekit::RemoteParticipant & participant)
  {
    for (const auto & [publication_sid, publication] : participant.trackPublications()) {
      (void)publication_sid;
      if (publication == nullptr || publication->kind() != livekit::TrackKind::KIND_VIDEO) {
        continue;
      }
      auto track = publication->track();
      if (track == nullptr) {
        continue;
      }
      if (track->stream_state() == livekit::StreamState::STATE_ACTIVE) {
        return true;
      }
    }
    return false;
  }

  livekit::RoomInfoData roomInfoSnapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (room_ == nullptr) {
      return livekit::RoomInfoData{};
    }
    return room_->room_info();
  }

  void logTokenState(const AccessToken & token)
  {
    if (token.refreshable) {
      LogEvent(kRoomSessionLogger, "room_token_loaded")
        .kv("phase", "connect")
        .kv("mode", "refreshable")
        .kv("exp_unix", formatTokenExpiry(token.expires_at))
        .info();
      return;
    }

    if (token.expires_at.has_value()) {
      LogEvent(kRoomSessionLogger, "room_token_loaded")
        .kv("phase", "connect")
        .kv("mode", "static")
        .kv("exp_unix", formatTokenExpiry(token.expires_at))
        .info();
      return;
    }

    LogEvent(kRoomSessionLogger, "room_token_expiry_unavailable")
      .kv("phase", "connect")
      .kv("mode", "static")
      .kv("reason", "parseable_exp_missing")
      .warn();
  }

  void waitForStop(std::chrono::milliseconds duration)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, duration, [this]() { return stop_requested_; });
  }

  bool stopRequested() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_requested_;
  }

  std::chrono::milliseconds initialBackoff() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return initial_backoff_;
  }

  std::chrono::milliseconds maxBackoff() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_backoff_;
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_thread_;
  std::shared_ptr<livekit::Room> room_;
  RoomConnectionConfig config_;
  std::shared_ptr<AccessTokenSource> token_source_;
  RoomSessionCallbacks callbacks_;
  std::unordered_map<std::string, RpcHandler> rpc_handlers_;
  std::unordered_map<const PublishedVideoTrack *, std::shared_ptr<livekit::LocalVideoTrack>> published_video_tracks_;
  AccessToken current_token_;
  std::chrono::milliseconds initial_backoff_{500};
  std::chrono::milliseconds max_backoff_{10000};
  std::chrono::seconds refresh_margin_{300};
  bool stop_requested_ = false;
  bool reconnect_requested_ = false;
  bool static_expiry_warned_ = false;
  bool livekit_initialized_ = false;
  // Guards runtime-facing disconnect callbacks so transient reconnect churn does not look like a
  // requester disappearing permanently.
  bool participant_disconnects_enabled_ = false;
  bool thread_started_ = false;
  std::string last_reconnect_reason_;
};

}  // namespace

StaticTokenSource::StaticTokenSource(std::string token)
: token_(std::move(token))
{}

AccessToken StaticTokenSource::getToken(const RoomConnectionConfig &)
{
  AccessToken token;
  token.value = token_;
  token.expires_at = parseJwtExpiresAt(token_);
  token.refreshable = false;
  return token;
}

ApiKeyAccessTokenSource::ApiKeyAccessTokenSource(std::string api_key, std::string api_secret, std::chrono::seconds ttl)
: api_key_(std::move(api_key))
, api_secret_(std::move(api_secret))
, ttl_(ttl)
{}

AccessToken ApiKeyAccessTokenSource::getToken(const RoomConnectionConfig & config)
{
  const auto now = Clock::now();
  LiveKitRoomGrant grant;
  grant.room = config.room;

  AccessToken token;
  token.value = mintLiveKitAccessToken(api_key_, api_secret_, config.identity, grant, now, ttl_);
  token.issued_at = now;
  token.expires_at = now + ttl_;
  token.refreshable = true;
  return token;
}

std::unique_ptr<RoomSession> makeRoomSession()
{
  return std::make_unique<LiveKitRoomSession>();
}

}  // namespace livekit_ros2_bridge
