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

#include "room_connection.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/livekit.h"
#include "livekit/local_data_track.h"
#include "livekit/local_participant.h"
#include "livekit/local_video_track.h"
#include "livekit/remote_participant.h"
#include "livekit/room_delegate.h"
#include "livekit/rpc_error.h"
#include "livekit/video_source.h"
#include "protocol/constants.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.room_connection");
constexpr char kLocalParticipantUnavailable[] = "LiveKit local participant unavailable.";

std::string_view stateName(livekit::ConnectionState state)
{
  switch (state) {
    case livekit::ConnectionState::Disconnected:
      return "disconnected";
    case livekit::ConnectionState::Connected:
      return "connected";
    case livekit::ConnectionState::Reconnecting:
      return "reconnecting";
  }
  return "unknown";
}

struct ParticipantRef
{
  std::shared_ptr<livekit::Room> room;
  livekit::LocalParticipant * participant = nullptr;
  std::uint64_t room_generation = 0;
};

class SdkRoomConnection final : public RoomConnection, private livekit::RoomDelegate
{
public:
  SdkRoomConnection() = default;

  ~SdkRoomConnection() override
  {
    stop();
  }

  void start(LiveKitConfig config, RoomEventCallbacks callbacks) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (connect_task_.joinable()) {
      return;
    }

    config_ = std::move(config);
    callbacks_ = std::move(callbacks);
    stop_requested_ = false;
    state_ = livekit::ConnectionState::Disconnected;
    connect_task_ = std::thread([this]() { run(); });
  }

  void stop() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!connect_task_.joinable()) {
      return;
    }
    stop_requested_ = true;
    lock.unlock();

    // Join outside mutex_; the connection task may reacquire it before exit.
    connect_task_.join();

    detachRoom();

    bool shutdown_sdk = false;
    {
      std::lock_guard<std::mutex> clear_lock(mutex_);
      callbacks_ = RoomEventCallbacks{};
      state_ = livekit::ConnectionState::Disconnected;
      shutdown_sdk = sdk_initialized_;
      sdk_initialized_ = false;
    }
    if (shutdown_sdk) {
      livekit::shutdown();
    }
  }

  bool registerRpc(const std::string & method, livekit::LocalParticipant::RpcHandler handler) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpc_handlers_[method] = std::move(handler);
    return registerRpcLocked(method);
  }

  bool unregisterRpc(const std::string & method) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rpc_handlers_.erase(method);

    auto * participant = room_ == nullptr ? nullptr : room_->localParticipant();
    if (participant == nullptr) {
      return true;
    }

    try {
      participant->unregisterRpcMethod(method);
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "rpc_method_unregistration_failed").field("method", method).field("error", exc.what()).error();
      return false;
    }
    return true;
  }

  void publishData(
    const std::vector<std::uint8_t> & payload,
    bool reliable,
    const std::vector<std::string> & destination_identities,
    const std::string & topic) override
  {
    const auto ref = participantRef();
    if (ref.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }
    ref.participant->publishData(payload, reliable, destination_identities, topic);
  }

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) override
  {
    const auto ref = participantRef();
    if (ref.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }
    auto result = ref.participant->publishDataTrack(name);
    if (!result) {
      throw std::runtime_error("Failed to publish data track '" + name + "': " + result.error().message);
    }
    return result.value();
  }

  livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, const livekit::DataTrackFrame & frame) override
  {
    return track->tryPush(frame);
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == livekit::ConnectionState::Disconnected) {
      return;
    }
    auto * participant = room_ == nullptr ? nullptr : room_->localParticipant();
    if (participant == nullptr) {
      return;
    }
    participant->unpublishDataTrack(track);
  }

  std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & options) override
  {
    if (name.empty()) {
      throw std::invalid_argument("Video track name is required.");
    }
    if (source == nullptr) {
      throw std::invalid_argument("Video source is required.");
    }

    const auto ref = participantRef();
    if (ref.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }

    auto track = livekit::LocalVideoTrack::createLocalVideoTrack(name, source);
    livekit::TrackPublishOptions publish_options = options;
    publish_options.source = livekit::TrackSource::SOURCE_CAMERA;
    ref.participant->publishTrack(track, publish_options);

    if (track == nullptr) {
      throw std::runtime_error("Failed to publish video track '" + name + "'.");
    }
    const auto publication = track->publication();
    if (publication == nullptr) {
      throw std::runtime_error("Failed to publish video track '" + name + "'.");
    }

    recordTrackIfCurrent(name, track, ref.room_generation);
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) override
  {
    if (track == nullptr) {
      return;
    }

    const std::string & name = track->name();
    try {
      unpublishVideoTrackIfCurrent(track);
    } catch (...) {
      try {
        LogEvent(kLogger, "video_track_unpublish_failed")
          .field("track_name", name)
          .fieldException("error", std::current_exception())
          .warn();
      } catch (...) {}
    }
  }

private:
  ParticipantRef participantRef() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ParticipantRef ref;
    ref.room = room_;
    ref.room_generation = room_generation_;
    if (state_ != livekit::ConnectionState::Disconnected && ref.room != nullptr) {
      ref.participant = ref.room->localParticipant();
    }
    return ref;
  }

  void unpublishVideoTrackIfCurrent(const std::shared_ptr<livekit::LocalVideoTrack> & track)
  {
    std::uint64_t room_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = track_room_generations_.find(track.get());
      if (it == track_room_generations_.end()) {
        return;
      }

      room_generation = it->second;
      track_room_generations_.erase(it);
    }

    const auto publication = track->publication();
    if (publication == nullptr) {
      return;
    }

    auto ref = participantRef();
    if (ref.participant == nullptr) {
      return;
    }
    if (ref.room_generation != room_generation) {
      return;
    }

    ref.participant->unpublishTrack(publication->sid());
  }

  void recordTrackIfCurrent(
    const std::string & name, const std::shared_ptr<livekit::LocalVideoTrack> & track, std::uint64_t room_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (room_generation != room_generation_) {
      // The room changed while publishTrack() was in flight; leave this stale track untracked.
      LogEvent(kLogger, "video_track_publish_stale").field("track_name", name).warn();
      return;
    }
    track_room_generations_[track.get()] = room_generation;
  }

  void run()
  {
    if (!livekit::initialize()) {
      LogEvent(kLogger, "livekit_initialize_failed").error();
      return;
    }

    bool should_connect = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_initialized_ = true;
      should_connect = !stop_requested_;
    }
    if (should_connect) {
      (void)connect();
    }
  }

  bool connect()
  {
    LiveKitConfig config;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      config = config_;
    }

    auto room = connectRoom(config);
    if (room == nullptr) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_requested_) {
        room->setDelegate(nullptr);
        return false;
      }
    }

    if (!activateRoom(std::move(room))) {
      detachRoom();
      return false;
    }

    transitionState(livekit::ConnectionState::Connected);
    return true;
  }

  std::shared_ptr<livekit::Room> connectRoom(const LiveKitConfig & config)
  {
    auto room = std::make_shared<livekit::Room>();
    room->setDelegate(this);

    const livekit::RoomOptions options;
    bool connected = false;
    try {
      connected = room->Connect(config.url, config.access_token, options);
      if (!connected) {
        LogEvent(kLogger, "room_connect_failed")
          .field("reason", "connect_returned_false")
          .fieldOr("url", config.url)
          .field("token_present", !config.access_token.empty())
          .error();
      }
    } catch (...) {
      LogEvent(kLogger, "room_connect_failed")
        .field("reason", "exception")
        .fieldOr("url", config.url)
        .field("token_present", !config.access_token.empty())
        .fieldException("error", std::current_exception())
        .error();
    }

    if (!connected) {
      room->setDelegate(nullptr);
      return nullptr;
    }

    if (room->localParticipant() == nullptr) {
      LogEvent(kLogger, "room_connect_failed")
        .field("reason", "local_participant_unavailable")
        .fieldOr("url", config.url)
        .field("token_present", !config.access_token.empty())
        .error();
      room->setDelegate(nullptr);
      return nullptr;
    }

    return room;
  }

  bool activateRoom(std::shared_ptr<livekit::Room> room)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++room_generation_;
    room_ = std::move(room);

    bool registered = true;
    for (const auto & entry : rpc_handlers_) {
      if (!registerRpcLocked(entry.first)) {
        registered = false;
      }
    }
    return registered;
  }

  void detachRoom()
  {
    std::shared_ptr<livekit::Room> detached_room;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      detached_room = std::move(room_);
      if (detached_room != nullptr) {
        ++room_generation_;
      }
      // Old-room tracks must not unpublish from the replacement room.
      track_room_generations_.clear();
      state_ = livekit::ConnectionState::Disconnected;
    }

    if (detached_room != nullptr) {
      detached_room->setDelegate(nullptr);
      detached_room.reset();
    }
  }

  void transitionState(livekit::ConnectionState state)
  {
    std::function<void(livekit::ConnectionState)> callback;
    std::string livekit_url;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == state) {
        return;
      }
      state_ = state;
      livekit_url = config_.url;
      callback = callbacks_.on_state_changed;
    }

    auto log = LogEvent(kLogger, "room_connection_state_changed").field("connection_state", stateName(state));
    if (state == livekit::ConnectionState::Connected) {
      log.fieldOr("url", livekit_url);
      log.info();
    } else {
      log.warn();
    }
    if (callback) {
      callback(state);
    }
  }

  void onParticipantDisconnected(livekit::Room &, const livekit::ParticipantDisconnectedEvent & event) override
  {
    const auto * participant = event.participant;
    if (participant == nullptr) {
      return;
    }

    std::function<void(const livekit::ParticipantDisconnectedEvent &)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ != livekit::ConnectionState::Connected) {
        return;
      }
      callback = callbacks_.on_participant_disconnected;
    }

    if (!callback) {
      return;
    }

    if (participant->identity().empty()) {
      return;
    }

    callback(event);
  }

  void onUserPacketReceived(livekit::Room &, const livekit::UserDataPacketEvent & event) override
  {
    std::function<void(const livekit::UserDataPacketEvent &)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == livekit::ConnectionState::Disconnected) {
        return;
      }
      callback = callbacks_.on_user_packet_received;
    }

    if (callback) {
      callback(event);
    }
  }

  void onConnectionStateChanged(livekit::Room &, const livekit::ConnectionStateChangedEvent & event) override
  {
    transitionState(event.state);
  }

  void onDisconnected(livekit::Room &, const livekit::DisconnectedEvent &) override
  {
    transitionState(livekit::ConnectionState::Disconnected);
  }

  void onReconnecting(livekit::Room &, const livekit::ReconnectingEvent &) override
  {
    transitionState(livekit::ConnectionState::Reconnecting);
  }

  void onReconnected(livekit::Room &, const livekit::ReconnectedEvent &) override
  {
    transitionState(livekit::ConnectionState::Connected);
  }

  void onRoomEos(livekit::Room &, const livekit::RoomEosEvent &) override
  {
    transitionState(livekit::ConnectionState::Disconnected);
  }

  bool registerRpcLocked(const std::string & method)
  {
    auto * participant = room_ == nullptr ? nullptr : room_->localParticipant();
    const auto it = rpc_handlers_.find(method);
    if (participant == nullptr || it == rpc_handlers_.end()) {
      return true;
    }

    try {
      participant->unregisterRpcMethod(method);
    } catch (const std::exception &) {
      // Re-registering refreshes any SDK-retained callback; absence is harmless.
    }

    try {
      // LiveKit retains this callback independently of rpc_handlers_.
      participant->registerRpcMethod(
        method,
        [method, handler = it->second](const livekit::RpcInvocationData & invocation) -> std::optional<std::string> {
          try {
            return handler(invocation);
          } catch (const livekit::RpcError &) {
            throw;
          } catch (...) {
            LogEvent(kLogger, "rpc_request_failed")
              .field("method", method)
              .fieldOr("request_id", invocation.request_id)
              .fieldOr("requester_identity", invocation.caller_identity)
              .fieldException("error", std::current_exception())
              .error();
            throw livekit::RpcError(protocol::kInternalRpcCode, "Internal error handling RPC method");
          }
        });
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "rpc_method_registration_failed").field("method", method).field("error", exc.what()).error();
      return false;
    }
    return true;
  }

  mutable std::mutex mutex_;
  std::thread connect_task_;

  std::shared_ptr<livekit::Room> room_;
  LiveKitConfig config_;
  RoomEventCallbacks callbacks_;

  std::unordered_map<std::string, livekit::LocalParticipant::RpcHandler> rpc_handlers_;
  // Guards video unpublish against tracks published by an older room.
  std::unordered_map<const livekit::LocalVideoTrack *, std::uint64_t> track_room_generations_;

  bool stop_requested_ = false;
  bool sdk_initialized_ = false;
  std::uint64_t room_generation_ = 0;
  livekit::ConnectionState state_ = livekit::ConnectionState::Disconnected;
};

}  // namespace

std::unique_ptr<RoomConnection> createRoomConnection()
{
  return std::make_unique<SdkRoomConnection>();
}

}  // namespace livekit_ros2_bridge
