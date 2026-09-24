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
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/data_stream.h"
#include "livekit/data_track_error.h"
#include "livekit/livekit.h"
#include "livekit/local_audio_track.h"
#include "livekit/local_data_track.h"
#include "livekit/local_participant.h"
#include "livekit/local_video_track.h"
#include "livekit/remote_participant.h"
#include "livekit/remote_track_publication.h"
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

struct ParticipantRef
{
  std::shared_ptr<livekit::Room> room;
  std::shared_ptr<livekit::LocalParticipant> participant;
  std::uint64_t room_generation = 0;
};

// The room whose event callback is running on this thread, or null. The SDK changes remote
// participants' publication maps only while handling room events, one at a time, so those maps are
// safe to read only from inside such a callback.
thread_local livekit::Room * current_event_room = nullptr;

// Marks this thread as inside a room-event callback for `room` until destroyed.
class RoomEventScope
{
public:
  explicit RoomEventScope(livekit::Room & room)
  : previous_room_(current_event_room)
  {
    current_event_room = &room;
  }

  ~RoomEventScope()
  {
    current_event_room = previous_room_;
  }

  RoomEventScope(const RoomEventScope &) = delete;
  RoomEventScope & operator=(const RoomEventScope &) = delete;

private:
  livekit::Room * previous_room_;
};

// The bridge follows the SDK's state once the room is activated and is Disconnected before that, so
// it never reports Connected while RPCs are unregistered.
livekit::ConnectionState bridgeConnectionState(livekit::ConnectionState sdk_state, bool room_activated)
{
  if (!room_activated) {
    return livekit::ConnectionState::Disconnected;
  }
  return sdk_state;
}

RemoteTrackEvent makeRemoteTrackEvent(
  const livekit::RemoteParticipant * participant,
  const std::shared_ptr<livekit::RemoteTrackPublication> & publication,
  const std::shared_ptr<livekit::Track> & track)
{
  RemoteTrackEvent remote_event;
  if (participant != nullptr) {
    remote_event.participant_identity = participant->identity();
  }
  if (publication != nullptr) {
    remote_event.track_sid = publication->sid();
    remote_event.track_name = publication->name();
    remote_event.track_kind = publication->kind();
  }
  if (track == nullptr) {
    return remote_event;
  }
  if (remote_event.track_sid.empty()) {
    remote_event.track_sid = track->sid();
    remote_event.track_kind = track->kind();
  }
  remote_event.track = track;
  return remote_event;
}

std::shared_ptr<livekit::RemoteTrackPublication> findRemotePublication(
  const livekit::Room & room, const std::string & participant_identity, const std::string & track_sid)
{
  const auto participant = room.remoteParticipant(participant_identity).lock();
  if (participant == nullptr) {
    return nullptr;
  }
  const auto & publications = participant->trackPublications();
  const auto publication = publications.find(track_sid);
  if (publication == publications.end()) {
    return nullptr;
  }
  return publication->second;
}

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

    auto participant = lockedLocalParticipant(room_);
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
      LogEvent(kLogger, "data_track_publish_failed")
        .fieldOr("track_name", name)
        .field("reason", "local_participant_unavailable")
        .warn();
      throw std::runtime_error(kLocalParticipantUnavailable);
    }

    auto result = ref.participant->publishDataTrack(name);
    if (!result) {
      const auto & error = result.error();
      LogEvent(kLogger, "data_track_publish_failed")
        .fieldOr("track_name", name)
        .fieldEnum("sdk_error_code", error.code)
        .fieldOr("error", error.message)
        .warn();
      throw std::runtime_error("Failed to publish data track '" + name + "': " + result.error().message);
    }

    auto track = result.value();
    if (track == nullptr) {
      LogEvent(kLogger, "data_track_publish_failed").fieldOr("track_name", name).field("reason", "null_track").warn();
      throw std::runtime_error("LiveKit returned a null data track.");
    }

    const auto & info = track->info();
    LogEvent(kLogger, "data_track_published").fieldOr("track_name", info.name).fieldOr("track_sid", info.sid).info();
    return track;
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
    if (state_ == livekit::ConnectionState::Disconnected) {
      return;
    }
    auto participant = lockedLocalParticipant(room_);
    if (participant == nullptr) {
      return;
    }
    const auto & info = track->info();
    try {
      participant->unpublishDataTrack(track);
    } catch (...) {
      LogEvent(kLogger, "data_track_unpublish_failed")
        .fieldOr("track_name", info.name)
        .fieldOr("track_sid", info.sid)
        .fieldException("error", std::current_exception())
        .warn();
      throw;
    }
    LogEvent(kLogger, "data_track_unpublished").fieldOr("track_name", info.name).fieldOr("track_sid", info.sid).info();
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

    try {
      auto track = livekit::LocalVideoTrack::createLocalVideoTrack(name, source);
      if (track == nullptr) {
        throw std::runtime_error("Failed to publish video track '" + name + "'.");
      }

      livekit::TrackPublishOptions publish_options = options;
      publish_options.source = livekit::TrackSource::SOURCE_CAMERA;
      ref.participant->publishTrack(track, publish_options);

      const auto publication = track->publication();
      if (publication == nullptr) {
        throw std::runtime_error("Failed to publish video track '" + name + "'.");
      }

      LogEvent(kLogger, "video_track_published")
        .fieldOr("track_sid", publication->sid())
        .fieldOr("track_name", publication->name())
        .info();

      recordTrackIfCurrent(name, track, ref.room_generation);
      return track;
    } catch (...) {
      LogEvent(kLogger, "video_track_publish_failed")
        .fieldOr("track_name", name)
        .field("track_width", source->width())
        .field("track_height", source->height())
        .fieldException("error", std::current_exception())
        .warn();
      throw;
    }
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
          .fieldOr("track_sid", track->sid())
          .fieldException("error", std::current_exception())
          .warn();
      } catch (...) {}
    }
  }

  std::shared_ptr<livekit::LocalAudioTrack> publishAudioTrack(
    const std::string & name,
    const std::shared_ptr<livekit::AudioSource> & source,
    const livekit::TrackPublishOptions & options) override
  {
    if (name.empty()) {
      throw std::invalid_argument("Audio track name is required.");
    }
    if (source == nullptr) {
      throw std::invalid_argument("Audio source is required.");
    }

    const auto ref = participantRef();
    if (ref.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }

    try {
      auto track = livekit::LocalAudioTrack::createLocalAudioTrack(name, source);
      if (track == nullptr) {
        throw std::runtime_error("Failed to publish audio track '" + name + "'.");
      }

      livekit::TrackPublishOptions publish_options = options;
      publish_options.source = livekit::TrackSource::SOURCE_MICROPHONE;
      ref.participant->publishTrack(track, publish_options);

      const auto publication = track->publication();
      if (publication == nullptr) {
        throw std::runtime_error("Failed to publish audio track '" + name + "'.");
      }

      LogEvent(kLogger, "audio_track_published")
        .fieldOr("track_sid", publication->sid())
        .fieldOr("track_name", publication->name())
        .info();

      recordAudioTrackIfCurrent(name, track, ref.room_generation);
      return track;
    } catch (...) {
      LogEvent(kLogger, "audio_track_publish_failed")
        .fieldOr("track_name", name)
        .field("track_sample_rate", source->sampleRate())
        .field("track_channels", source->numChannels())
        .fieldException("error", std::current_exception())
        .warn();
      throw;
    }
  }

  void unpublishAudioTrack(const std::shared_ptr<livekit::LocalAudioTrack> & track) override
  {
    if (track == nullptr) {
      return;
    }

    const std::string & name = track->name();
    try {
      unpublishAudioTrackIfCurrent(track);
    } catch (...) {
      try {
        LogEvent(kLogger, "audio_track_unpublish_failed")
          .field("track_name", name)
          .fieldOr("track_sid", track->sid())
          .fieldException("error", std::current_exception())
          .warn();
      } catch (...) {}
    }
  }

  bool subscribeRemoteTrack(const std::string & participant_identity, const std::string & track_sid) override
  {
    if (participant_identity.empty() || track_sid.empty()) {
      return false;
    }
    if (current_event_room == nullptr) {
      LogEvent(kLogger, "remote_track_subscribe_failed")
        .field("reason", "outside_room_event")
        .fieldOr("participant_identity", participant_identity)
        .fieldOr("track_sid", track_sid)
        .error();
      return false;
    }

    const auto publication = findRemotePublication(*current_event_room, participant_identity, track_sid);
    if (publication == nullptr) {
      LogEvent(kLogger, "remote_track_subscribe_failed")
        .field("reason", "publication_unavailable")
        .fieldOr("participant_identity", participant_identity)
        .fieldOr("track_sid", track_sid)
        .warn();
      return false;
    }
    if (publication->subscribed()) {
      return true;
    }

    // setSubscribed() is a blocking FFI request; never hold mutex_ across it.
    try {
      publication->setSubscribed(true);
    } catch (const std::exception & exception) {
      LogEvent(kLogger, "remote_track_subscribe_failed")
        .fieldOr("participant_identity", participant_identity)
        .fieldOr("track_sid", track_sid)
        .field("error", exception.what())
        .warn();
      return false;
    }
    return true;
  }

  std::vector<RoomConnection::RemoteTrackSnapshotEntry> remoteTrackSnapshot() override
  {
    std::vector<RoomConnection::RemoteTrackSnapshotEntry> entries;
    if (current_event_room == nullptr) {
      LogEvent(kLogger, "remote_track_snapshot_failed").field("reason", "outside_room_event").error();
      return entries;
    }

    for (const auto & remote_handle : current_event_room->remoteParticipants()) {
      const auto participant = remote_handle.lock();
      if (participant == nullptr) {
        continue;
      }
      for (const auto & [track_sid, publication] : participant->trackPublications()) {
        if (publication == nullptr || track_sid.empty()) {
          continue;
        }
        entries.push_back(
          RoomConnection::RemoteTrackSnapshotEntry{
            participant->identity(), track_sid, publication->name(), publication->kind(), publication->subscribed()});
      }
    }
    return entries;
  }

  void sendByteStream(
    const std::string & topic,
    const std::string & name,
    const std::string & content_type,
    std::shared_ptr<const std::vector<std::uint8_t>> payload,
    const std::string & destination_identity) override
  {
    // Defend the interface: callers dispatch only when a cached value exists, so a null buffer is a
    // caller bug. Reject it before spawning a thread, and as an invalid argument (not a silent skip)
    // so the caller can't mistake "nothing sent" for a successful send.
    if (payload == nullptr) {
      throw std::invalid_argument("Byte-stream payload is required.");
    }

    const auto ref = participantRef();
    if (ref.participant == nullptr) {
      throw std::runtime_error(kLocalParticipantUnavailable);
    }

    // STOPGAP — converge with the broader uncancellable-blocking-.get() sweep.
    //
    // livekit::ByteStreamWriter::write() sends each chunk through an uncancellable blocking SDK
    // call. On robot networks these block far more often than the SDK surface suggests, and a
    // stalled client can hang it indefinitely. Running write()/close() on the ROS executor or a
    // LiveKit callback thread would therefore freeze live data relay, heartbeats, and every other
    // ROS callback. So the whole construct + write() + close() runs on a *detached, sacrificial*
    // thread that holds a strong reference to the room (keeping the local participant alive for the
    // transfer). A hung client wedges a sacrificial thread instead of a load-bearing one.
    //
    // This is a deliberately localized fix; it must merge into the systemic blocking-call sweep so
    // both land on one policy. Do not move write()/close() back onto a caller thread.
    std::thread([room = ref.room, topic, name, content_type, payload, destination_identity]() {
      // One backstop for the whole body: the ByteStreamWriter constructor, the write, and both
      // close() calls are uncancellable FFI that throw (per the SDK header) on transfer errors and
      // teardown races. An exception escaping a detached thread calls std::terminate() and aborts
      // the process, so — like RosExecutorQueue::drain() — nothing is allowed past this boundary,
      // and every failure is logged here exactly once.
      try {
        auto participant = lockedLocalParticipant(room);
        if (participant == nullptr) {
          LogEvent(kLogger, "byte_stream_send_skipped")
            .field("topic", topic)
            .field("reason", "local_participant_unavailable")
            .warn();
          return;
        }

        livekit::ByteStreamWriter writer(
          *participant, name, topic, {}, "", payload->size(), content_type, {destination_identity});
        try {
          writer.write(*payload);
          writer.close();
        } catch (...) {
          // The writer does not close on destruction; an unterminated stream leaves the remote
          // reader waiting forever, so close with a reason before letting the boundary below log it.
          try {
            writer.close("send failed");
          } catch (...) {}
          throw;
        }
      } catch (...) {
        LogEvent(kLogger, "byte_stream_send_failed")
          .field("topic", topic)
          .field("destination_identity", destination_identity)
          .fieldException("error", std::current_exception())
          .warn();
      }
    }).detach();
  }

private:
  // Requires mutex_ to be held (or to be passed a room snapshotted under it).
  static std::shared_ptr<livekit::LocalParticipant> lockedLocalParticipant(const std::shared_ptr<livekit::Room> & room)
  {
    return room == nullptr ? std::shared_ptr<livekit::LocalParticipant>{} : room->localParticipant().lock();
  }

  ParticipantRef participantRef() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ParticipantRef ref;
    ref.room = room_;
    ref.room_generation = room_generation_;
    if (state_ != livekit::ConnectionState::Disconnected) {
      ref.participant = lockedLocalParticipant(ref.room);
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
    LogEvent(kLogger, "video_track_unpublished")
      .fieldOr("track_name", publication->name())
      .fieldOr("track_sid", publication->sid())
      .info();
  }

  void recordTrackIfCurrent(
    const std::string & name, const std::shared_ptr<livekit::LocalVideoTrack> & track, std::uint64_t room_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (room_generation != room_generation_) {
      // The room changed while publishTrack() was in flight; leave this stale track untracked.
      LogEvent(kLogger, "video_track_publish_stale")
        .field("track_name", name)
        .fieldOr("track_sid", track->sid())
        .warn();
      return;
    }
    track_room_generations_[track.get()] = room_generation;
  }

  void unpublishAudioTrackIfCurrent(const std::shared_ptr<livekit::LocalAudioTrack> & track)
  {
    std::uint64_t room_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = audio_track_room_generations_.find(track.get());
      if (it == audio_track_room_generations_.end()) {
        return;
      }

      room_generation = it->second;
      audio_track_room_generations_.erase(it);
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
    LogEvent(kLogger, "audio_track_unpublished")
      .fieldOr("track_name", publication->name())
      .fieldOr("track_sid", publication->sid())
      .info();
  }

  void recordAudioTrackIfCurrent(
    const std::string & name, const std::shared_ptr<livekit::LocalAudioTrack> & track, std::uint64_t room_generation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (room_generation != room_generation_) {
      // The room changed while publishTrack() was in flight; leave this stale track untracked.
      LogEvent(kLogger, "audio_track_publish_stale")
        .field("track_name", name)
        .fieldOr("track_sid", track->sid())
        .warn();
      return;
    }
    audio_track_room_generations_[track.get()] = room_generation;
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

    bool stop_requested = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested = stop_requested_;
    }
    if (stop_requested) {
      abandonRoom(*room);
      return false;
    }

    auto active_room = room;
    if (!activateRoom(std::move(room))) {
      // The room was never activated, so the bridge never left Disconnected and nothing is reported.
      detachRoom();
      return false;
    }

    LogEvent(kLogger, "room_connected")
      .fieldOr("url", config.url)
      .fieldOr("room_sid", active_room->roomInfo().sid)
      .fieldOr("room_name", active_room->roomInfo().name)
      .info();
    // Reports Connected now if the SDK's Connected already arrived; otherwise that event will.
    reportBridgeState();
    return true;
  }

  std::shared_ptr<livekit::Room> connectRoom(const LiveKitConfig & config)
  {
    auto room = std::make_shared<livekit::Room>();
    room->setDelegate(this);

    // The bridge subscribes only to tracks it names; room-wide media
    // reception was never a contract. Track events still arrive for publications
    // the bridge deliberately subscribes to.
    livekit::RoomOptions options;
    options.auto_subscribe = false;

    // Set before connect(): the SDK can deliver this room's events before activateRoom() runs.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      event_room_ = room.get();
      sdk_state_ = livekit::ConnectionState::Disconnected;
    }

    bool connected = false;
    try {
      connected = room->connect(config.url, config.access_token, options);
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
      abandonRoom(*room);
      return nullptr;
    }

    if (room->localParticipant().lock() == nullptr) {
      LogEvent(kLogger, "room_connect_failed")
        .field("reason", "local_participant_unavailable")
        .fieldOr("url", config.url)
        .field("token_present", !config.access_token.empty())
        .error();
      abandonRoom(*room);
      return nullptr;
    }

    return room;
  }

  // Detaches a room that never became room_. It was never activated, so the bridge is still
  // Disconnected and its late events can no longer report anything.
  void abandonRoom(livekit::Room & room)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      event_room_ = nullptr;
      sdk_state_ = livekit::ConnectionState::Disconnected;
    }
    room.setDelegate(nullptr);
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
    room_activated_ = registered;
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
      event_room_ = nullptr;
      room_activated_ = false;
      sdk_state_ = livekit::ConnectionState::Disconnected;
      state_ = livekit::ConnectionState::Disconnected;
    }

    if (detached_room != nullptr) {
      detached_room->setDelegate(nullptr);
      detached_room.reset();
    }
  }

  // Applies an SDK connection event from `room`; events from any room but the current one are
  // ignored. Entering Connected fires on_remote_tracks_ready in this same callback, before the SDK
  // can deliver a later track event, so catching up and forwarding new publishes cannot interleave.
  void updateSdkState(livekit::Room & room, livekit::ConnectionState sdk_state)
  {
    const RoomEventScope scope(room);
    std::function<void()> ready_callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (&room != event_room_ || sdk_state_ == sdk_state) {
        return;
      }
      sdk_state_ = sdk_state;
      if (sdk_state == livekit::ConnectionState::Connected) {
        ready_callback = callbacks_.on_remote_tracks_ready;
      }
    }

    reportBridgeState();
    if (ready_callback) {
      ready_callback();
    }
  }

  // Reports a change in the bridge state derived from sdk_state_ and room_activated_. The connect
  // thread and the SDK thread both report, so state_report_mutex_ keeps their reports in order.
  void reportBridgeState()
  {
    const std::lock_guard<std::mutex> report_lock(state_report_mutex_);
    std::function<void(livekit::ConnectionState)> callback;
    livekit::ConnectionState state = livekit::ConnectionState::Disconnected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = bridgeConnectionState(sdk_state_, room_activated_);
      if (state_ == state) {
        return;
      }
      state_ = state;
      callback = callbacks_.on_state_changed;
    }

    if (callback) {
      callback(state);
    }
  }

  // Copies the callback under mutex_: stop() reassigns callbacks_ under it, and the SDK does not wait
  // for an in-flight delegate call when the delegate is detached. Returns an empty callback for events
  // from a stale room, while the SDK is Disconnected, or (with `connected_only`) while it is not
  // Connected. Gating on the SDK state keeps events that arrive before activation.
  template <typename CallbackT>
  CallbackT currentCallback(
    const livekit::Room & room, CallbackT RoomEventCallbacks::* callback_member, bool connected_only) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (&room != event_room_ || sdk_state_ == livekit::ConnectionState::Disconnected) {
      return {};
    }
    if (connected_only && sdk_state_ != livekit::ConnectionState::Connected) {
      return {};
    }
    return callbacks_.*callback_member;
  }

  void forwardRemoteTrackEvent(
    const livekit::Room & room,
    std::function<void(const RemoteTrackEvent &)> RoomEventCallbacks::* callback_member,
    const RemoteTrackEvent & remote_event)
  {
    const auto callback = currentCallback(room, callback_member, false);
    if (callback) {
      callback(remote_event);
    }
  }

  // Forwarded only while the SDK is Connected, since SDK reconnects suppress transient disconnects.
  void onParticipantDisconnected(livekit::Room & room, const livekit::ParticipantDisconnectedEvent & event) override
  {
    const RoomEventScope scope(room);
    if (event.participant == nullptr || event.participant->identity().empty()) {
      return;
    }
    const auto callback = currentCallback(room, &RoomEventCallbacks::on_participant_disconnected, true);
    if (callback) {
      callback(event);
    }
  }

  // Forwarded only while the SDK is Connected: the on_remote_tracks_ready snapshot covers anything
  // published before it, including the tracks a full restart re-announces while Reconnecting.
  void onTrackPublished(livekit::Room & room, const livekit::TrackPublishedEvent & event) override
  {
    const RoomEventScope scope(room);
    if (event.participant == nullptr) {
      return;
    }
    const auto callback = currentCallback(room, &RoomEventCallbacks::on_remote_track_published, true);
    if (!callback) {
      return;
    }

    if (event.publication != nullptr && !event.publication->sid().empty()) {
      callback(makeRemoteTrackEvent(event.participant, event.publication, nullptr));
      return;
    }

    // SDK v1.6.0 kTrackPublished moves the publication into the participant's map and then hands the
    // delegate the moved-from (null) pointer. Announce all of the participant's publications instead;
    // subscribers match by name and skip tracks already subscribed.
    std::vector<RemoteTrackEvent> announced;
    for (const auto & [track_sid, publication] : event.participant->trackPublications()) {
      if (publication == nullptr || track_sid.empty()) {
        continue;
      }
      announced.push_back(makeRemoteTrackEvent(event.participant, publication, nullptr));
    }
    for (const auto & remote_event : announced) {
      callback(remote_event);
    }
  }

  void onTrackUnpublished(livekit::Room & room, const livekit::TrackUnpublishedEvent & event) override
  {
    const RoomEventScope scope(room);
    forwardRemoteTrackEvent(
      room,
      &RoomEventCallbacks::on_remote_track_unpublished,
      makeRemoteTrackEvent(event.participant, event.publication, nullptr));
  }

  void onTrackSubscribed(livekit::Room & room, const livekit::TrackSubscribedEvent & event) override
  {
    const RoomEventScope scope(room);
    forwardRemoteTrackEvent(
      room,
      &RoomEventCallbacks::on_remote_track_subscribed,
      makeRemoteTrackEvent(event.participant, event.publication, event.track));
  }

  void onTrackUnsubscribed(livekit::Room & room, const livekit::TrackUnsubscribedEvent & event) override
  {
    const RoomEventScope scope(room);
    forwardRemoteTrackEvent(
      room,
      &RoomEventCallbacks::on_remote_track_unsubscribed,
      makeRemoteTrackEvent(event.participant, event.publication, event.track));
  }

  void onTrackSubscriptionFailed(livekit::Room & room, const livekit::TrackSubscriptionFailedEvent & event) override
  {
    const RoomEventScope scope(room);
    const auto callback = currentCallback(room, &RoomEventCallbacks::on_remote_track_subscription_failed, false);
    if (!callback) {
      return;
    }

    RemoteTrackSubscriptionFailedEvent translated;
    if (event.participant != nullptr) {
      translated.participant_identity = event.participant->identity();
    }
    translated.track_sid = event.track_sid;
    translated.error = event.error;

    callback(translated);
  }

  void onRoomSidChanged(livekit::Room & room, const livekit::RoomSidChangedEvent & event) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (room_.get() != &room || state_ == livekit::ConnectionState::Disconnected) {
        return;
      }
    }

    LogEvent(kLogger, "room_sid_changed").fieldOr("room_sid", event.sid).info();
  }

  void onRoomMoved(livekit::Room &, const livekit::RoomMovedEvent & event) override
  {
    LogEvent(kLogger, "room_moved").fieldOr("room_sid", event.info.sid).info();
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

  void onConnectionStateChanged(livekit::Room & room, const livekit::ConnectionStateChangedEvent & event) override
  {
    updateSdkState(room, event.state);
  }

  void onDisconnected(livekit::Room & room, const livekit::DisconnectedEvent & event) override
  {
    LogEvent(kLogger, "room_disconnected").fieldEnum("disconnect_reason", event.reason).warn();
    updateSdkState(room, livekit::ConnectionState::Disconnected);
  }

  void onReconnecting(livekit::Room & room, const livekit::ReconnectingEvent &) override
  {
    LogEvent(kLogger, "room_reconnecting").fieldOr("room_sid", room.roomInfo().sid).warn();
    updateSdkState(room, livekit::ConnectionState::Reconnecting);
  }

  void onReconnected(livekit::Room & room, const livekit::ReconnectedEvent &) override
  {
    LogEvent(kLogger, "room_reconnected").fieldOr("room_sid", room.roomInfo().sid).info();
    updateSdkState(room, livekit::ConnectionState::Connected);
  }

  void onRoomEos(livekit::Room & room, const livekit::RoomEosEvent &) override
  {
    LogEvent(kLogger, "room_eos").warn();
    updateSdkState(room, livekit::ConnectionState::Disconnected);
  }

  bool registerRpcLocked(const std::string & method)
  {
    auto participant = lockedLocalParticipant(room_);
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
  // Guards audio unpublish against tracks published by an older room.
  std::unordered_map<const livekit::LocalAudioTrack *, std::uint64_t> audio_track_room_generations_;
  // The room whose delegate events are accepted; compared, never dereferenced.
  const livekit::Room * event_room_ = nullptr;

  bool stop_requested_ = false;
  bool sdk_initialized_ = false;
  // True between a successful activateRoom() and detachRoom().
  bool room_activated_ = false;
  std::uint64_t room_generation_ = 0;
  // The SDK's state for event_room_; gates remote-track events.
  livekit::ConnectionState sdk_state_ = livekit::ConnectionState::Disconnected;
  // The bridge state reported through on_state_changed; see bridgeConnectionState().
  livekit::ConnectionState state_ = livekit::ConnectionState::Disconnected;
  // Taken before mutex_, never while holding it.
  std::mutex state_report_mutex_;
};

}  // namespace

std::unique_ptr<RoomConnection> createRoomConnection()
{
  return std::make_unique<SdkRoomConnection>();
}

}  // namespace livekit_ros2_bridge
