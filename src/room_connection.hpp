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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "livekit/data_track_frame.h"
#include "livekit/local_participant.h"
#include "livekit/room_event_types.h"
#include "livekit/track.h"

namespace livekit
{
class LocalAudioTrack;
class LocalDataTrack;
class LocalVideoTrack;
struct LocalDataTrackTryPushError;
template <typename T, typename E>
class Result;
struct ParticipantDisconnectedEvent;
struct UserDataPacketEvent;
class VideoSource;
}  // namespace livekit

namespace livekit_ros2_bridge
{

struct LiveKitConfig
{
  std::string url;
  std::string access_token;
};

struct RoomEventCallbacks
{
  // Bridge connection state. Connected means the SDK is connected and the room is activated (RPCs
  // registered); each change is reported once, in order.
  std::function<void(livekit::ConnectionState)> on_state_changed;

  // Fires inside the SDK's Connected/Reconnected event, before any later remote-track event, so a
  // handler can catch up from remoteTrackSnapshot(). May fire before the bridge reports Connected.
  std::function<void()> on_remote_tracks_ready;

  // Runs on a connection-managed thread, not necessarily a ROS executor thread.
  std::function<void(const livekit::UserDataPacketEvent &)> on_user_packet_received;

  // SDK reconnect suppresses transient disconnects; LiveKit owns the event lifetime.
  std::function<void(const livekit::ParticipantDisconnectedEvent &)> on_participant_disconnected;

  // Remote media track events, translated out of SDK publication objects so
  // handlers never depend on publication lifetimes. Forwarded only after remote
  // tracks are ready and until the SDK disconnects; published events only while
  // it is Connected. `track` is set only on subscribed/unsubscribed events.
  std::function<void(const struct RemoteTrackEvent &)> on_remote_track_published;
  std::function<void(const struct RemoteTrackEvent &)> on_remote_track_unpublished;
  std::function<void(const struct RemoteTrackEvent &)> on_remote_track_subscribed;
  std::function<void(const struct RemoteTrackEvent &)> on_remote_track_unsubscribed;
  std::function<void(const struct RemoteTrackSubscriptionFailedEvent &)> on_remote_track_subscription_failed;
};

// Plain remote-media-track event the connection derives from SDK track events.
// `track` is the media track handle on subscribed/unsubscribed events and null
// on published/unpublished events (no media exists before subscription).
struct RemoteTrackEvent
{
  std::string participant_identity;
  std::string track_sid;
  std::string track_name;
  livekit::TrackKind track_kind = livekit::TrackKind::KIND_UNKNOWN;
  std::shared_ptr<livekit::Track> track;
};

// Plain event the connection derives from the SDK's TrackSubscriptionFailedEvent.
// Carries no publication object; the bridge reports and cleans up, never retries.
struct RemoteTrackSubscriptionFailedEvent
{
  std::string participant_identity;
  std::string track_sid;
  std::string error;
};

// Thread-safe facade around one SDK-owned room; callbacks may run on connection-managed threads.
class RoomConnection
{
public:
  virtual ~RoomConnection() = default;

  // Starts a background connection task. Repeated calls are ignored until stop() returns.
  virtual void start(LiveKitConfig config, RoomEventCallbacks callbacks) = 0;

  // Stops the active room and waits for the connection task to exit.
  virtual void stop() = 0;

  // A false return means active SDK registration failed; the handler remains saved for the next connection.
  virtual bool registerRpc(const std::string & method, livekit::LocalParticipant::RpcHandler handler) = 0;

  virtual bool unregisterRpc(const std::string & method) = 0;

  // Publish calls require an active local participant and may throw while disconnected.
  virtual void publishData(
    const std::vector<std::uint8_t> & payload,
    bool reliable = true,
    const std::vector<std::string> & destination_identities = {},
    const std::string & topic = {}) = 0;

  virtual std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string & name) = 0;

  virtual livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> & track, const livekit::DataTrackFrame & frame) = 0;

  virtual void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> & track) = 0;

  // Returned video tracks carry SDK publication identity; stale-track unpublishes are no-ops.
  virtual std::shared_ptr<livekit::LocalVideoTrack> publishVideoTrack(
    const std::string & name,
    const std::shared_ptr<livekit::VideoSource> & source,
    const livekit::TrackPublishOptions & options) = 0;

  virtual void unpublishVideoTrack(const std::shared_ptr<livekit::LocalVideoTrack> & track) = 0;

  // Returned audio tracks carry SDK publication identity; stale-track unpublishes are no-ops.
  virtual std::shared_ptr<livekit::LocalAudioTrack> publishAudioTrack(
    const std::string & name,
    const std::shared_ptr<livekit::AudioSource> & source,
    const livekit::TrackPublishOptions & options) = 0;

  virtual void unpublishAudioTrack(const std::shared_ptr<livekit::LocalAudioTrack> & track) = 0;

  // subscribeRemoteTrack() and remoteTrackSnapshot() read the SDK's publication state, so they work
  // only from inside on_remote_tracks_ready or a remote-track callback; elsewhere they log and return
  // false or an empty snapshot.

  // Requests media delivery for one already-published remote track by identity. A false return
  // means the subscription request could not be issued; the subscriber may retry on a later event.
  virtual bool subscribeRemoteTrack(const std::string & participant_identity, const std::string & track_sid) = 0;

  // Snapshots the remote participants' media-track publications currently in the room so a
  // re-subscriber can recover state from after a (re)connect. Returns one entry per published
  // remote media track, including tracks the bridge has not subscribed to.
  struct RemoteTrackSnapshotEntry
  {
    std::string participant_identity;
    std::string track_sid;
    std::string track_name;
    livekit::TrackKind track_kind = livekit::TrackKind::KIND_UNKNOWN;
    bool subscribed = false;
  };

  virtual std::vector<RemoteTrackSnapshotEntry> remoteTrackSnapshot() = 0;

  // Send raw bytes as a targeted byte stream addressed to exactly one participant.
  // `topic` is the fixed stream topic (e.g. lkros.echo.once); `name` is the per-delivery label the
  // recipient reads to route the stream (e.g. the requested ROS topic). `payload` is a shared,
  // immutable buffer (typically aliased from the publisher's cached last message), so dispatch never
  // copies the bytes. Non-blocking: the actual SDK write runs on a detached thread, so a slow/hung
  // client never blocks the caller. See the implementation for the threading rationale.
  // Each call spawns one detached sender; sends are not rate-limited, so callers are responsible for
  // any throttling. Throws synchronously if the payload is null or the local participant is
  // unavailable; a transfer failure after handoff is logged by the sender, not thrown.
  virtual void sendByteStream(
    const std::string & topic,
    const std::string & name,
    const std::string & content_type,
    std::shared_ptr<const std::vector<std::uint8_t>> payload,
    const std::string & destination_identity) = 0;
};

std::unique_ptr<RoomConnection> createRoomConnection();

}  // namespace livekit_ros2_bridge
