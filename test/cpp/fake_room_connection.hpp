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
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livekit/remote_participant.h"
#include "livekit/room_event_types.h"
#include "room_connection.hpp"

namespace livekit_ros2_bridge
{

constexpr char kUnknownDataTrackName[] = "<unknown>";

struct PushedDataTrackFrame
{
  std::string track_name;
  std::vector<std::uint8_t> payload;
};

struct FakeRoomConnectionState
{
  RoomEventCallbacks callbacks;
  bool started = false;
  bool stopped = false;
  std::string access_token;

  std::vector<std::string> registered_rpc_methods;
  std::vector<std::string> unregistered_rpc_methods;
  std::vector<std::string> event_log;
  std::vector<OutgoingPacket> published_outgoing_packets;

  std::vector<std::string> published_data_track_names;
  std::vector<PushedDataTrackFrame> pushed_data_track_frames;
  std::vector<std::string> unpublished_data_track_names;
  std::vector<std::string> published_video_track_names;
  std::vector<VideoPublishConfig> published_video_configs;
  std::vector<std::string> unpublished_video_track_names;
  std::vector<std::string> unpublish_attempted_data_track_names;
  std::vector<std::string> unpublish_rejected_data_track_names;

  std::vector<std::string> rejected_rpc_methods;
  std::map<std::string, RpcHandler> rpc_handlers;

  std::function<void(const RoomEventCallbacks & callbacks)> stop_hook;
  std::function<std::shared_ptr<livekit::LocalDataTrack>(const std::string & name)> publish_data_track_handler;
  std::function<DataTrackPushResult(const std::string & name, const std::vector<std::uint8_t> & payload)>
    try_push_data_track_handler;

  bool throw_on_publish_packet = false;
  int publish_packet_call_count = 0;
};

inline std::vector<std::uint8_t> userPacketPayloadBytes(const std::string & payload)
{
  return std::vector<std::uint8_t>(payload.begin(), payload.end());
}

inline void emitUserPacket(
  const RoomEventCallbacks & callbacks,
  std::vector<std::uint8_t> payload,
  std::string topic,
  const std::string & requester_identity)
{
  if (!callbacks.on_user_packet_received) {
    return;
  }

  livekit::UserDataPacketEvent event;
  event.data = std::move(payload);
  event.topic = std::move(topic);

  if (requester_identity.empty()) {
    callbacks.on_user_packet_received(event);
    return;
  }

  livekit::RemoteParticipant participant(
    livekit::FfiHandle{},
    "fake-participant-sid",
    "fake-participant-name",
    requester_identity,
    "",
    std::unordered_map<std::string, std::string>{},
    livekit::ParticipantKind::Standard,
    livekit::DisconnectReason::Unknown);
  event.participant = &participant;
  callbacks.on_user_packet_received(event);
}

inline void emitUserPacket(
  const RoomEventCallbacks & callbacks,
  const std::string & payload,
  std::string topic,
  const std::string & requester_identity)
{
  emitUserPacket(callbacks, userPacketPayloadBytes(payload), std::move(topic), requester_identity);
}

class FakePublishedVideoTrack final : public PublishedVideoTrack
{
public:
  FakePublishedVideoTrack(std::string name, std::shared_ptr<FakeRoomConnectionState> state)
  : PublishedVideoTrack(std::move(name))
  , state_(std::move(state))
  {}

  ~FakePublishedVideoTrack() noexcept override
  {
    try {
      if (state_ == nullptr) {
        return;
      }
      state_->event_log.push_back("unpublish_video_track:" + name());
      state_->unpublished_video_track_names.push_back(name());
    } catch (...) {}
  }

private:
  std::shared_ptr<FakeRoomConnectionState> state_;
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
    state->started = true;
    state->access_token = std::move(config.access_token);
    state->callbacks = std::move(callbacks);
  }

  bool registerRpc(const std::string & method_name, RpcHandler handler) override
  {
    state->registered_rpc_methods.push_back(method_name);
    const bool registration_rejected =
      std::find(state->rejected_rpc_methods.begin(), state->rejected_rpc_methods.end(), method_name) !=
      state->rejected_rpc_methods.end();
    if (registration_rejected) {
      return false;
    }
    state->rpc_handlers[method_name] = std::move(handler);
    return true;
  }

  bool unregisterRpc(const std::string & method_name) override
  {
    state->event_log.push_back("unregister:" + method_name);
    state->unregistered_rpc_methods.push_back(method_name);
    state->rpc_handlers.erase(method_name);
    return true;
  }

  void publishPacket(const OutgoingPacket & packet) override
  {
    state->publish_packet_call_count++;
    if (state->throw_on_publish_packet) {
      throw std::runtime_error("simulated publishPacket failure");
    }
    state->event_log.push_back("publish_packet:" + packet.topic);
    state->published_outgoing_packets.push_back(packet);
  }

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) override
  {
    state->event_log.push_back("publish_data_track:" + name);
    state->published_data_track_names.push_back(name);
    auto track = state->publish_data_track_handler ? state->publish_data_track_handler(name) : makeSyntheticDataTrack();
    if (track != nullptr) {
      data_track_names_[track.get()] = name;
    }
    return track;
  }

  DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, std::vector<std::uint8_t> payload) override
  {
    const std::string name = lookupDataTrackName(track);
    state->event_log.push_back("push_data_track:" + name);
    const auto result = state->try_push_data_track_handler ? state->try_push_data_track_handler(name, payload)
                                                           : DataTrackPushResult::success();
    if (result) {
      state->pushed_data_track_frames.push_back({name, std::move(payload)});
    }
    return result;
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    const std::string name = lookupDataTrackName(track);
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

  std::unique_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const VideoPublishConfig & config) override
  {
    (void)source;
    state->event_log.push_back("publish_video_track:" + name);
    state->published_video_track_names.push_back(name);
    state->published_video_configs.push_back(config);
    return std::make_unique<FakePublishedVideoTrack>(name, state);
  }

  void stop() override
  {
    if (state->stopped) {
      return;
    }

    state->stopped = true;
    state->event_log.push_back("stop");
    if (state->stop_hook) {
      state->stop_hook(state->callbacks);
    }
  }

  void emitConnectionReset() const
  {
    if (state->callbacks.on_connection_reset) {
      state->callbacks.on_connection_reset();
    }
  }

  void emitConnected() const
  {
    if (state->callbacks.on_connected) {
      state->callbacks.on_connected();
    }
  }

  void emitReconnectRequested(const std::string & reason) const
  {
    if (state->callbacks.on_reconnect_requested) {
      state->callbacks.on_reconnect_requested(reason);
    }
  }

  void emitReconnecting(const std::string & reason) const
  {
    if (state->callbacks.on_reconnecting) {
      state->callbacks.on_reconnecting(reason);
    }
  }

  void emitReconnected() const
  {
    if (state->callbacks.on_reconnected) {
      state->callbacks.on_reconnected();
    }
  }

  void emitParticipantDisconnected(const std::string & requester_identity) const
  {
    if (state->callbacks.on_remote_participant_disconnected) {
      state->callbacks.on_remote_participant_disconnected(requester_identity);
    }
  }

  void emitUserPacket(
    std::vector<std::uint8_t> payload, std::string topic, const std::string & requester_identity) const
  {
    livekit_ros2_bridge::emitUserPacket(state->callbacks, std::move(payload), std::move(topic), requester_identity);
  }

  void emitUserPacket(const std::string & payload, std::string topic, const std::string & requester_identity) const
  {
    livekit_ros2_bridge::emitUserPacket(state->callbacks, payload, std::move(topic), requester_identity);
  }

  std::shared_ptr<FakeRoomConnectionState> state;

private:
  std::shared_ptr<livekit::LocalDataTrack> makeSyntheticDataTrack()
  {
    auto owner = std::make_shared<int>(next_track_id_++);
    return std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
  }

  std::string lookupDataTrackName(const std::shared_ptr<livekit::LocalDataTrack> & track) const
  {
    const auto it = data_track_names_.find(track.get());
    return it == data_track_names_.end() ? kUnknownDataTrackName : it->second;
  }

  int next_track_id_ = 1;
  std::map<const livekit::LocalDataTrack *, std::string> data_track_names_;
};

}  // namespace livekit_ros2_bridge
