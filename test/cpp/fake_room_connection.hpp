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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/data_track_frame.h"
#include "livekit/remote_participant.h"
#include "livekit/result.h"
#include "livekit/room_event_types.h"
#include "room_connection.hpp"

namespace livekit_ros2_bridge
{

constexpr char kUnknownDataTrackName[] = "<unknown>";

struct PushedDataTrackFrame
{
  std::string track_name;
  livekit::DataTrackFrame frame;
};

struct PublishedDataCall
{
  std::vector<std::uint8_t> payload;
  bool reliable = true;
  std::vector<std::string> destination_identities;
  std::string topic;
};

class FakeRoomConnection;

struct FakeRoomConnectionState
{
  RoomEventCallbacks callbacks;
  livekit::ConnectionState connection_state = livekit::ConnectionState::Disconnected;
  bool started = false;
  bool stopped = false;
  std::string access_token;

  std::vector<std::string> registered_rpc_methods;
  std::vector<std::string> unregistered_rpc_methods;
  std::vector<std::string> event_log;
  std::vector<PublishedDataCall> published_data_calls;

  std::vector<std::string> published_data_track_names;
  std::vector<PushedDataTrackFrame> pushed_data_track_frames;
  std::vector<std::string> unpublished_data_track_names;
  std::vector<std::string> published_video_track_names;
  std::vector<livekit::TrackPublishOptions> published_video_options;
  std::vector<std::string> unpublished_video_track_names;
  std::vector<std::string> unpublish_attempted_data_track_names;
  std::vector<std::string> unpublish_rejected_data_track_names;

  std::vector<std::string> rejected_rpc_methods;
  std::map<std::string, livekit::LocalParticipant::RpcHandler> rpc_handlers;

  std::function<void(FakeRoomConnection & connection)> stop_hook;
  std::function<std::shared_ptr<livekit::LocalDataTrack>(const std::string & name)> publish_data_track_handler;
  std::function<livekit::Result<void, livekit::LocalDataTrackTryPushError>(
    const std::string & name, const livekit::DataTrackFrame & frame)>
    try_push_data_track_handler;
  std::function<void(const std::string & name)> publish_video_track_hook;
  std::function<void(const std::string & name)> unpublish_video_track_hook;

  bool throw_on_publish_data = false;
  bool throw_on_unpublish_video = false;
  int publish_data_call_count = 0;

  std::size_t publishedVideoTrackCount() const
  {
    return published_video_track_count.load(std::memory_order_acquire);
  }

  void notePublishedVideoTrack()
  {
    published_video_track_count.store(published_video_track_names.size(), std::memory_order_release);
  }

private:
  std::atomic<std::size_t> published_video_track_count{0};
};

class FakeRoomConnection final : public RoomConnection
{
public:
  FakeRoomConnection()
  : state(std::make_shared<FakeRoomConnectionState>())
  {}

  ~FakeRoomConnection() override
  {
    stop();
  }

  void start(LiveKitConfig config, RoomEventCallbacks callbacks) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state->started = true;
    state->access_token = std::move(config.access_token);
    state->callbacks = std::move(callbacks);
    state->connection_state = livekit::ConnectionState::Disconnected;
  }

  bool registerRpc(const std::string & method, livekit::LocalParticipant::RpcHandler handler) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state->registered_rpc_methods.push_back(method);
    const bool registration_rejected =
      std::find(state->rejected_rpc_methods.begin(), state->rejected_rpc_methods.end(), method) !=
      state->rejected_rpc_methods.end();
    if (registration_rejected) {
      return false;
    }
    state->rpc_handlers[method] = std::move(handler);
    return true;
  }

  bool unregisterRpc(const std::string & method) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state->event_log.push_back("unregister:" + method);
    state->unregistered_rpc_methods.push_back(method);
    state->rpc_handlers.erase(method);
    return true;
  }

  void publishData(
    const std::vector<std::uint8_t> & payload,
    bool reliable,
    const std::vector<std::string> & destination_identities,
    const std::string & topic) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state->publish_data_call_count++;
    if (state->throw_on_publish_data) {
      throw std::runtime_error("simulated publishData failure");
    }
    state->event_log.push_back("publish_data:" + topic);
    state->published_data_calls.push_back(PublishedDataCall{payload, reliable, destination_identities, topic});
  }

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) override
  {
    std::function<std::shared_ptr<livekit::LocalDataTrack>(const std::string & name)> handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->event_log.push_back("publish_data_track:" + name);
      state->published_data_track_names.push_back(name);
      handler = state->publish_data_track_handler;
    }

    auto track = handler ? handler(name) : makeSyntheticDataTrack();
    if (track != nullptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      data_track_names_[track.get()] = name;
    }
    return track;
  }

  livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, const livekit::DataTrackFrame & frame) override
  {
    const std::string name = lookupDataTrackName(track);
    std::function<livekit::Result<void, livekit::LocalDataTrackTryPushError>(
      const std::string & name, const livekit::DataTrackFrame & frame)>
      handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->event_log.push_back("push_data_track:" + name);
      handler = state->try_push_data_track_handler;
    }

    const auto result =
      handler ? handler(name, frame) : livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
    if (result) {
      std::lock_guard<std::mutex> lock(mutex_);
      state->pushed_data_track_frames.push_back({name, frame});
    }
    return result;
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    const std::string name = lookupDataTrackName(track);
    std::lock_guard<std::mutex> lock(mutex_);
    state->unpublish_attempted_data_track_names.push_back(name);
    const bool unpublish_rejected =
      std::find(
        state->unpublish_rejected_data_track_names.begin(), state->unpublish_rejected_data_track_names.end(), name) !=
      state->unpublish_rejected_data_track_names.end();
    if (unpublish_rejected) {
      throw std::runtime_error("simulated unpublish failure");
    }
    state->event_log.push_back("unpublish_data_track");
    state->unpublished_data_track_names.push_back(name);
    data_track_names_.erase(track.get());
  }

  std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & options) override
  {
    (void)source;
    std::function<void(const std::string & name)> hook;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->event_log.push_back("publish_video_track:" + name);
      state->published_video_track_names.push_back(name);
      state->published_video_options.push_back(options);
      state->notePublishedVideoTrack();
      hook = state->publish_video_track_hook;
    }
    if (hook) {
      hook(name);
    }

    auto track = makeSyntheticVideoTrack();
    std::lock_guard<std::mutex> lock(mutex_);
    video_track_names_[track.get()] = name;
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) override
  {
    const std::string name = lookupVideoTrackName(track);
    std::function<void(const std::string & name)> hook;
    bool throw_on_unpublish_video = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->event_log.push_back("unpublish_video_track:" + name);
      state->unpublished_video_track_names.push_back(name);
      video_track_names_.erase(track.get());
      hook = state->unpublish_video_track_hook;
      throw_on_unpublish_video = state->throw_on_unpublish_video;
    }
    if (hook) {
      hook(name);
    }
    if (throw_on_unpublish_video) {
      throw std::runtime_error("simulated video unpublish failure");
    }
  }

  void stop() override
  {
    std::function<void(FakeRoomConnection & connection)> hook;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state->stopped) {
        return;
      }

      state->stopped = true;
      state->event_log.push_back("stop");
      hook = state->stop_hook;
    }

    if (hook) {
      hook(*this);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->callbacks = RoomEventCallbacks{};
      state->connection_state = livekit::ConnectionState::Disconnected;
    }
  }

  void emitConnected() const
  {
    emitConnectionState(livekit::ConnectionState::Connected);
  }

  void emitDisconnected() const
  {
    emitConnectionState(livekit::ConnectionState::Disconnected);
  }

  void emitRoomEos() const
  {
    emitConnectionState(livekit::ConnectionState::Disconnected);
  }

  void emitReconnecting() const
  {
    emitConnectionState(livekit::ConnectionState::Reconnecting);
  }

  void emitReconnected() const
  {
    emitConnectionState(livekit::ConnectionState::Connected);
  }

  void emitParticipantDisconnected(const std::string & requester_identity) const
  {
    std::function<void(const livekit::ParticipantDisconnectedEvent &)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (
        state->connection_state != livekit::ConnectionState::Connected || requester_identity.empty() ||
        !state->callbacks.on_participant_disconnected)
      {
        return;
      }
      callback = state->callbacks.on_participant_disconnected;
    }

    livekit::ParticipantDisconnectedEvent event;
    auto participant = makeRemoteParticipant(requester_identity);
    event.participant = &participant;
    callback(event);
  }

  void emitUserPacket(
    std::vector<std::uint8_t> payload, std::string topic, const std::string & requester_identity) const
  {
    std::function<void(const livekit::UserDataPacketEvent &)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!state->callbacks.on_user_packet_received) {
        return;
      }
      callback = state->callbacks.on_user_packet_received;
    }

    livekit::UserDataPacketEvent event;
    event.data = std::move(payload);
    event.topic = std::move(topic);

    if (requester_identity.empty()) {
      callback(event);
      return;
    }

    auto participant = makeRemoteParticipant(requester_identity);
    event.participant = &participant;
    callback(event);
  }

  void emitUserPacket(const std::string & payload, std::string topic, const std::string & requester_identity) const
  {
    emitUserPacket(std::vector<std::uint8_t>(payload.begin(), payload.end()), std::move(topic), requester_identity);
  }

  std::shared_ptr<livekit::LocalDataTrack> makeSyntheticDataTrack()
  {
    auto owner = std::make_shared<int>(next_track_id_.fetch_add(1, std::memory_order_relaxed));
    return std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
  }

  std::shared_ptr<livekit::LocalVideoTrack> makeSyntheticVideoTrack()
  {
    auto owner = std::make_shared<int>(next_track_id_.fetch_add(1, std::memory_order_relaxed));
    return std::shared_ptr<livekit::LocalVideoTrack>(owner, reinterpret_cast<livekit::LocalVideoTrack *>(owner.get()));
  }

  std::shared_ptr<FakeRoomConnectionState> state;

private:
  void emitConnectionState(livekit::ConnectionState new_state) const
  {
    std::function<void(livekit::ConnectionState)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state->connection_state == new_state) {
        return;
      }
      state->connection_state = new_state;
      callback = state->callbacks.on_state_changed;
    }
    if (callback) {
      callback(new_state);
    }
  }

  static livekit::RemoteParticipant makeRemoteParticipant(std::string identity)
  {
    return livekit::RemoteParticipant(
      livekit::FfiHandle{},
      "fake-participant-sid",
      "fake-participant-name",
      std::move(identity),
      "",
      std::unordered_map<std::string, std::string>{},
      livekit::ParticipantKind::Standard,
      livekit::DisconnectReason::Unknown);
  }

  std::string lookupDataTrackName(const std::shared_ptr<livekit::LocalDataTrack> & track) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = data_track_names_.find(track.get());
    return it == data_track_names_.end() ? kUnknownDataTrackName : it->second;
  }

  std::string lookupVideoTrackName(const std::shared_ptr<livekit::LocalVideoTrack> & track) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = video_track_names_.find(track.get());
    return it == video_track_names_.end() ? kUnknownDataTrackName : it->second;
  }

  mutable std::mutex mutex_;
  std::atomic<int> next_track_id_{1};
  std::map<const livekit::LocalDataTrack *, std::string> data_track_names_;
  std::map<const livekit::LocalVideoTrack *, std::string> video_track_names_;
};

}  // namespace livekit_ros2_bridge
