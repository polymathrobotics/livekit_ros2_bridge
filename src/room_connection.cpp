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

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
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
#include "livekit/local_participant.h"
#include "livekit/local_video_track.h"
#include "livekit/rpc_error.h"
#include "livekit/video_source.h"
#include "livekit_room_delegate.hpp"
#include "protocol/constants.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kReconnectInitialBackoff = std::chrono::milliseconds(500);
constexpr auto kReconnectMaxBackoff = std::chrono::milliseconds(10000);

}  // namespace

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.room_connection");
constexpr char kUnknownLogValue[] = "<unknown>";
constexpr char kLocalParticipantUnavailable[] = "LiveKit local participant unavailable.";

struct LocalParticipantSnapshot
{
  // Keeps the SDK room alive while callers use participant after releasing mutex_.
  std::shared_ptr<livekit::Room> room;
  livekit::LocalParticipant * participant = nullptr;
  std::uint64_t room_generation = 0;
};

struct VideoTrackEntry
{
  std::uint64_t room_generation = 0;
};

// Owns one reconnect worker thread. Public API calls, delegate control callbacks, and reconnect
// teardown all synchronize through mutex_.
class LiveKitRoomConnection final : public RoomConnection
{
public:
  LiveKitRoomConnection() = default;

  ~LiveKitRoomConnection() override
  {
    stop();
  }

  void start(LiveKitConfig config, LiveKitRoomDelegate & delegate) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_thread_.joinable()) {
      return;
    }

    config_ = std::move(config);
    delegate_ = &delegate;
    delegate_->setReconnectRequestHandler([this](const std::string & reason) { return requestReconnect(reason); });
    stop_requested_ = false;
    reconnect_reason_.reset();
    worker_thread_ = std::thread([this]() { run(); });
  }

  void stop() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!worker_thread_.joinable()) {
      return;
    }
    LiveKitRoomDelegate * delegate = delegate_;
    // Wake the worker regardless of whether it is currently waiting on an active connection or
    // sleeping in reconnect backoff; the stop path reuses the same condition variable.
    stop_requested_ = true;
    condition_.notify_all();
    lock.unlock();

    // Join outside mutex_ because run() reacquires it during teardown before the thread exits.
    worker_thread_.join();

    if (delegate != nullptr) {
      delegate->clearReconnectRequestHandler();
    }
    std::lock_guard<std::mutex> clear_lock(mutex_);
    if (delegate_ == delegate) {
      delegate_ = nullptr;
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
    const auto snapshot = snapshotLocalParticipant();
    if (snapshot.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }
    snapshot.participant->publishData(payload, reliable, destination_identities, topic);
  }

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) override
  {
    const auto snapshot = snapshotLocalParticipant();
    if (snapshot.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }
    auto result = snapshot.participant->publishDataTrack(name);
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
    if (track == nullptr) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto * participant = room_ == nullptr ? nullptr : room_->localParticipant();
    if (participant == nullptr) {
      return;
    }
    participant->unpublishDataTrack(track);
  }

  std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & config) override
  {
    if (name.empty()) {
      throw std::invalid_argument("Video track name is required.");
    }
    if (source == nullptr) {
      throw std::invalid_argument("Video source is required.");
    }

    const auto snapshot = snapshotLocalParticipant();
    if (snapshot.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }

    auto track = livekit::LocalVideoTrack::createLocalVideoTrack(name, source);
    livekit::TrackPublishOptions options = config;
    options.source = livekit::TrackSource::SOURCE_CAMERA;
    snapshot.participant->publishTrack(track, options);
    const auto publication = track == nullptr ? nullptr : track->publication();
    if (publication == nullptr) {
      throw std::runtime_error("Failed to publish video track '" + name + "'.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot.room_generation != room_generation_) {
      // reset() invalidated this room after publishTrack() returned, so the caller gets
      // a track that is already stale and safely degrades to a later no-op.
      LogEvent(kLogger, "video_track_publish_stale")
        .field("track_name", name)
        .field("track_sid", publication->sid())
        .field("reason", "room_reset")
        .warn();
      return track;
    }
    video_tracks_[track.get()] = VideoTrackEntry{snapshot.room_generation};
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) override
  {
    if (track == nullptr) {
      return;
    }

    const std::string & track_name = track->name();
    VideoTrackEntry entry;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = video_tracks_.find(track.get());
      if (it == video_tracks_.end()) {
        return;
      }
      entry = std::move(it->second);
      video_tracks_.erase(it);
    }

    try {
      const auto publication = track->publication();
      if (publication == nullptr) {
        return;
      }

      const auto snapshot = snapshotLocalParticipant();
      if (snapshot.participant == nullptr || snapshot.room_generation != entry.room_generation) {
        return;
      }
      snapshot.participant->unpublishTrack(publication->sid());
    } catch (...) {
      try {
        LogEvent(kLogger, "video_track_unpublish_failed")
          .field("track_name", track_name)
          .fieldException("error", std::current_exception())
          .warn();
      } catch (...) {}
    }
  }

private:
  LocalParticipantSnapshot snapshotLocalParticipant() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    LocalParticipantSnapshot snapshot;
    // Keep the Room alive after releasing the mutex because participant-facing SDK calls borrow
    // room-owned state and can block while reconnect/teardown clears room_ on another thread.
    snapshot.room = room_;
    snapshot.room_generation = room_generation_;
    snapshot.participant = snapshot.room == nullptr ? nullptr : snapshot.room->localParticipant();
    return snapshot;
  }

  void run()
  {
    if (!livekit::initialize()) {
      LogEvent(kLogger, "livekit_initialize_failed").field("reason", "initialize_returned_false").error();
      return;
    }

    auto backoff = initial_backoff_;
    while (true) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_requested_) {
          break;
        }
      }

      const bool connected = connect();
      if (connected) {
        backoff = initial_backoff_;
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return stop_requested_ || reconnect_reason_.has_value(); });
      }

      bool should_stop = false;
      std::string reason;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        should_stop = stop_requested_;
        reason = reconnect_reason_.value_or("retry_backoff");
      }

      reset(connected && !should_stop);

      if (should_stop) {
        break;
      }

      if (backoff.count() > 0) {
        LogEvent(kLogger, "room_reconnect_backoff")
          .field("reason", reason.c_str())
          .field("delay_seconds", backoff.count() / 1000.0)
          .warn();
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, backoff, [this]() { return stop_requested_; });
      }
      backoff = std::min(backoff * 2, max_backoff_);
    }

    reset(false);
    livekit::shutdown();
  }

  bool connect()
  {
    LiveKitConfig config;
    LiveKitRoomDelegate * delegate = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      config = config_;
      delegate = delegate_;
    }

    if (delegate == nullptr) {
      LogEvent(kLogger, "room_connect_failed").field("reason", "delegate_unavailable").error();
      return false;
    }

    auto room = std::make_shared<livekit::Room>();
    room->setDelegate(delegate);

    const livekit::RoomOptions options;

    try {
      if (!room->Connect(config.url, config.access_token, options)) {
        LogEvent(kLogger, "room_connect_failed")
          .field("reason", "connect_returned_false")
          .field("url", config.url)
          .field("token_present", !config.access_token.empty())
          .error();
        // This Room only exists for the current connect attempt, so clear the delegate before
        // dropping it on every failure path.
        room->setDelegate(nullptr);
        return false;
      }
    } catch (...) {
      LogEvent(kLogger, "room_connect_failed")
        .field("reason", "exception")
        .field("url", config.url)
        .field("token_present", !config.access_token.empty())
        .fieldException("error", std::current_exception())
        .error();

      room->setDelegate(nullptr);
      return false;
    }

    auto * participant = room->localParticipant();
    if (participant == nullptr) {
      LogEvent(kLogger, "room_connect_failed")
        .field("reason", "local_participant_unavailable")
        .field("url", config.url)
        .field("token_present", !config.access_token.empty())
        .error();
      room->setDelegate(nullptr);
      return false;
    }

    bool registered = true;
    const livekit::RoomInfoData info = room->room_info();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Publish the newly connected room before replaying RPC registrations so registerRpcLocked()
      // binds against this connection's local participant under the same mutex.
      ++room_generation_;
      room_ = std::move(room);
      reconnect_reason_.reset();
      for (const auto & entry : rpc_handlers_) {
        if (!registerRpcLocked(entry.first)) {
          registered = false;
        }
      }
    }

    if (!registered) {
      LogEvent(kLogger, "rpc_registration_incomplete").error();
      reset(false);
      return false;
    }

    const std::string identity = participant->identity();
    LogEvent(kLogger, "room_connected")
      .fieldOr("room", info.name, kUnknownLogValue)
      .fieldOr("sid", info.sid.has_value() ? info.sid->c_str() : nullptr, kUnknownLogValue)
      .fieldOr("identity", identity, kUnknownLogValue)
      .info();
    delegate->roomConnected();
    return true;
  }

  void reset(bool notify)
  {
    std::shared_ptr<livekit::Room> room;
    LiveKitRoomDelegate * delegate = nullptr;
    std::string reason;
    std::size_t dropped_tracks = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      room = std::move(room_);
      if (room != nullptr) {
        ++room_generation_;
      }
      // Published video handles are scoped to the old room's SDK objects. Drop them eagerly so
      // stale handles become harmless no-ops after reconnect.
      dropped_tracks = video_tracks_.size();
      video_tracks_.clear();
      reason = reconnect_reason_.value_or("");
      reconnect_reason_.reset();
      delegate = delegate_;
    }

    if (room != nullptr) {
      room->setDelegate(nullptr);
      room.reset();
    }

    if (!notify) {
      return;
    }

    LogEvent(kLogger, "room_connection_reset")
      .field("reason", reason.empty() ? "connection_reset" : reason.c_str())
      .fieldIf(dropped_tracks > 0U, "dropped_video_tracks", dropped_tracks)
      .info();

    if (delegate != nullptr) {
      delegate->roomConnectionReset();
    }
  }

  bool requestReconnect(const std::string & reason)
  {
    bool already_requested = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Keep the first reconnect cause for this episode; later transport noise should not
      // overwrite the reason reported during reset/backoff logging.
      already_requested = reconnect_reason_.has_value();
      if (!already_requested) {
        reconnect_reason_ = reason;
      }
      condition_.notify_all();
    }

    return !already_requested;
  }

  bool registerRpcLocked(const std::string & method)
  {
    auto * participant = room_ == nullptr ? nullptr : room_->localParticipant();
    const auto handler_it = rpc_handlers_.find(method);
    if (participant == nullptr || handler_it == rpc_handlers_.end()) {
      return true;
    }

    try {
      participant->unregisterRpcMethod(method);
    } catch (const std::exception &) {
      // Ignore missing-method errors; we only need a clean re-register.
    }

    try {
      // Capture the current handler by value because LiveKit retains this callback independently
      // of rpc_handlers_; unregister/teardown must not leave the SDK with dangling references.
      participant->registerRpcMethod(
        method,
        [method,
         handler = handler_it->second](const livekit::RpcInvocationData & invocation) -> std::optional<std::string> {
          try {
            return handler(invocation);
          } catch (const livekit::RpcError &) {
            throw;
          } catch (...) {
            LogEvent(kLogger, "rpc_request_failed")
              .field("method", method)
              .fieldOr("request_id", invocation.request_id, kUnknownLogValue)
              .fieldOr("requester_identity", invocation.caller_identity, kUnknownLogValue)
              .fieldException("error", std::current_exception())
              .error();
            throw livekit::RpcError(protocol::kInternalRpcError, "Internal error handling RPC method");
          }
        });
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "rpc_method_registration_failed").field("method", method).field("error", exc.what()).error();
      return false;
    }
    return true;
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_thread_;

  std::shared_ptr<livekit::Room> room_;
  LiveKitConfig config_;
  LiveKitRoomDelegate * delegate_ = nullptr;

  std::unordered_map<std::string, livekit::LocalParticipant::RpcHandler> rpc_handlers_;
  // Indexed by the SDK local track shared with callers. The room generation keeps late
  // publish/unpublish completions from crossing a reconnect boundary.
  std::unordered_map<const livekit::LocalVideoTrack *, VideoTrackEntry> video_tracks_;

  std::chrono::milliseconds initial_backoff_{kReconnectInitialBackoff};
  std::chrono::milliseconds max_backoff_{kReconnectMaxBackoff};

  bool stop_requested_ = false;
  // Bumped whenever the active room changes or is cleared so in-flight video operations cannot
  // repopulate handle state across reconnect boundaries.
  std::uint64_t room_generation_ = 0;
  // Presence means a reconnect episode is pending; the stored string carries the first cause.
  std::optional<std::string> reconnect_reason_;
};

}  // namespace

std::unique_ptr<RoomConnection> createRoomConnection()
{
  return std::make_unique<LiveKitRoomConnection>();
}

}  // namespace livekit_ros2_bridge
