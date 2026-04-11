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
#include <utility>
#include <vector>

#include "room_session.hpp"

namespace livekit_ros2_bridge
{

constexpr char kUnknownTrackName[] = "<unknown>";

struct FakeRoomSessionState
{
  RoomSessionCallbacks callbacks;
  bool started = false;
  bool stopped = false;
  std::string access_token;
  std::vector<std::string> registered_rpc_methods;
  std::vector<std::string> unregistered_rpc_methods;
  std::vector<std::string> event_log;
  std::vector<OutgoingControlPacket> published_outgoing_control_packets;
  std::vector<std::string> published_cdr_track_names;
  std::vector<std::string> unpublished_cdr_track_names;
  std::vector<std::string> published_video_track_names;
  std::vector<VideoPublishConfig> published_video_configs;
  std::vector<std::string> unpublished_video_track_names;
  std::vector<std::string> attempted_cdr_track_unpublish_names;
  std::vector<std::string> rejected_cdr_track_unpublish_names;
  std::vector<std::string> rejected_rpc_methods;
  std::map<std::string, RpcHandler> rpc_handlers;
  std::function<void(const RoomSessionCallbacks & callbacks)> stop_hook;
  std::function<std::shared_ptr<livekit::LocalDataTrack>(const std::string & name)> publish_cdr_track_handler;
  bool throw_on_publish_control_packet = false;
  int publish_control_packet_call_count = 0;
};

class FakeRoomSession final : public RoomSession
{
public:
  FakeRoomSession()
  : state(std::make_shared<FakeRoomSessionState>())
  {}

  void start(
    RoomConnectionConfig config,
    std::string access_token,
    RoomSessionCallbacks callbacks,
    std::chrono::milliseconds initial_backoff,
    std::chrono::milliseconds max_backoff) override
  {
    (void)config;
    (void)initial_backoff;
    (void)max_backoff;
    state->started = true;
    state->access_token = std::move(access_token);
    state->callbacks = std::move(callbacks);
  }

  bool registerRpcMethod(const std::string & method_name, RpcHandler handler) override
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

  bool unregisterRpcMethod(const std::string & method_name) override
  {
    state->event_log.push_back("unregister:" + method_name);
    state->unregistered_rpc_methods.push_back(method_name);
    state->rpc_handlers.erase(method_name);
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket & packet) override
  {
    state->publish_control_packet_call_count++;
    if (state->throw_on_publish_control_packet) {
      throw std::runtime_error("simulated publishControlPacket failure");
    }
    state->event_log.push_back("publish_control_packet:" + packet.control_topic);
    state->published_outgoing_control_packets.push_back(packet);
  }

  std::shared_ptr<livekit::LocalDataTrack> publishCdrTrack(const std::string & name) override
  {
    state->event_log.push_back("publish_cdr_track:" + name);
    state->published_cdr_track_names.push_back(name);
    auto track = state->publish_cdr_track_handler ? state->publish_cdr_track_handler(name) : makeSyntheticTrack();
    if (track != nullptr) {
      cdr_track_names_[track.get()] = name;
    }
    return track;
  }

  void unpublishCdrTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) override
  {
    const std::string name = lookupTrackName(track);
    state->attempted_cdr_track_unpublish_names.push_back(name);
    const bool unpublish_rejected =
      std::find(
        state->rejected_cdr_track_unpublish_names.begin(), state->rejected_cdr_track_unpublish_names.end(), name) !=
      state->rejected_cdr_track_unpublish_names.end();
    if (unpublish_rejected) {
      throw std::runtime_error("simulated unpublish failure");
    }
    state->event_log.push_back("unpublish_cdr_track");
    state->unpublished_cdr_track_names.push_back(name);
    cdr_track_names_.erase(track.get());
  }

  std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const VideoPublishConfig & publish_config) override
  {
    (void)source;
    state->event_log.push_back("publish_video_track:" + track_name);
    state->published_video_track_names.push_back(track_name);
    state->published_video_configs.push_back(publish_config);
    auto track = std::make_shared<PublishedVideoTrack>();
    track->track_name = track_name;
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) override
  {
    if (track == nullptr) {
      return;
    }
    state->event_log.push_back("unpublish_video_track:" + track->track_name);
    state->unpublished_video_track_names.push_back(track->track_name);
  }

  void stop() override
  {
    state->stopped = true;
    state->event_log.push_back("stop");
    if (state->stop_hook) {
      state->stop_hook(state->callbacks);
    }
  }

  void emitSessionReset() const
  {
    if (state->callbacks.on_session_reset) {
      state->callbacks.on_session_reset();
    }
  }

  void emitParticipantDisconnected(const std::string & requester_identity) const
  {
    if (state->callbacks.on_participant_disconnected) {
      state->callbacks.on_participant_disconnected(requester_identity);
    }
  }

  void emitIncomingControlPacket(const IncomingControlPacket & packet) const
  {
    if (state->callbacks.on_incoming_control_packet_received) {
      state->callbacks.on_incoming_control_packet_received(packet);
    }
  }

  std::shared_ptr<FakeRoomSessionState> state;

private:
  std::shared_ptr<livekit::LocalDataTrack> makeSyntheticTrack()
  {
    auto owner = std::make_shared<int>(next_track_id_++);
    return std::shared_ptr<livekit::LocalDataTrack>(owner, reinterpret_cast<livekit::LocalDataTrack *>(owner.get()));
  }

  std::string lookupTrackName(const std::shared_ptr<livekit::LocalDataTrack> & track) const
  {
    const auto it = cdr_track_names_.find(track.get());
    return it == cdr_track_names_.end() ? kUnknownTrackName : it->second;
  }

  int next_track_id_ = 1;
  std::map<const livekit::LocalDataTrack *, std::string> cdr_track_names_;
};

}  // namespace livekit_ros2_bridge
