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

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "audio/audio_output_manager.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "livekit/audio_frame.h"
#include "livekit/audio_stream.h"
#include "livekit/participant.h"
#include "livekit/remote_participant.h"
#include "livekit/track.h"
#include "protocol/constants.hpp"
#include "ros_test_support.hpp"

namespace livekit_ros2_bridge::audio
{
namespace
{

// Tests assert external behavior only — track events in, subscribe calls and
// reader lifecycle out — never pipeline internals. The reader threads consume a
// fake AudioOutputStream and bind a fake AudioOutputSink, so the tests drive
// frames synchronously without an audio device or the LiveKit FFI. The manager
// is a plain object: tests call its handlers directly, or route the fake
// connection's events into them, the way Runtime's callback wiring does.

constexpr char kOutputTrackName[] = "lkros.audio.out";
constexpr char kTestSinkFragment[] = "fakesink sync=false";

// Returned as a prvalue: RemoteParticipant is neither copyable nor movable,
// so callers must let guaranteed copy elision construct it in place and keep
// the local alive while the event points into it. A shared static here would
// freeze the identity at the first call and silently cross-couple tests.
livekit::RemoteParticipant makeDisconnectedParticipant(const std::string & identity)
{
  return livekit::RemoteParticipant(
    livekit::FfiHandle{},
    "fake-participant-sid",
    "fake-participant-name",
    identity,
    "",
    std::unordered_map<std::string, std::string>{},
    livekit::ParticipantKind::Standard,
    livekit::DisconnectReason::Unknown);
}

livekit::ParticipantDisconnectedEvent makeDisconnectedEvent(livekit::RemoteParticipant & participant)
{
  livekit::ParticipantDisconnectedEvent event;
  event.participant = &participant;
  return event;
}

RemoteTrackEvent subscribedOutputEvent(
  const std::string & participant_identity, const std::shared_ptr<livekit::Track> & track)
{
  return RemoteTrackEvent{participant_identity, track->sid(), kOutputTrackName, livekit::TrackKind::KIND_AUDIO, track};
}

RemoteTrackEvent unpublishedOutputEvent(const std::string & participant_identity, const std::string & track_sid)
{
  return RemoteTrackEvent{participant_identity, track_sid, kOutputTrackName, livekit::TrackKind::KIND_AUDIO, nullptr};
}

// Routes the fake connection's room events into the manager the way
// Runtime::makeRoomCallbacks does, so the fake's publication snapshot and the
// manager see the same SDK event sequence. The callbacks capture the manager
// by reference, so nothing may be emitted after it is destroyed.
void routeRoomEvents(FakeRoomConnection & connection, AudioOutputManager & manager)
{
  RoomEventCallbacks callbacks;
  callbacks.on_remote_tracks_ready = [&manager]() { manager.onConnected(); };
  callbacks.on_participant_disconnected = [&manager](const livekit::ParticipantDisconnectedEvent & event) {
    manager.onParticipantDisconnected(event);
  };
  callbacks.on_remote_track_published = [&manager](const RemoteTrackEvent & event) {
    manager.onRemoteTrackPublished(event);
  };
  callbacks.on_remote_track_unpublished = [&manager](const RemoteTrackEvent & event) {
    manager.onRemoteTrackUnpublished(event);
  };
  callbacks.on_remote_track_subscribed = [&manager](const RemoteTrackEvent & event) {
    manager.onRemoteTrackSubscribed(event);
  };
  callbacks.on_remote_track_unsubscribed = [&manager](const RemoteTrackEvent & event) {
    manager.onRemoteTrackUnsubscribed(event);
  };
  connection.start(LiveKitConfig{}, std::move(callbacks));
}

// Single-owner sink mirroring AudioOutputSink's claim semantics: bind succeeds only
// while unclaimed; only the owner's pushes land. Records every call so tests can
// assert the reader handover and cleanup behavior.
class FakeAudioOutputSink final : public AudioOutputSinkInterface
{
public:
  struct BindCall
  {
    std::uint64_t reader_id;
    int sample_rate;
    int channels;
  };

  struct PushCall
  {
    std::uint64_t reader_id;
    std::size_t count;
  };

  bool bind(std::uint64_t reader_id, int sample_rate, int num_channels) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bind_calls_.push_back(BindCall{reader_id, sample_rate, num_channels});
    if (owner_ != 0) {
      return false;
    }
    owner_ = reader_id;
    return true;
  }

  void push(std::uint64_t reader_id, const std::int16_t * samples, std::size_t count) override
  {
    (void)samples;
    std::lock_guard<std::mutex> lock(mutex_);
    if (owner_ != reader_id) {
      return;
    }
    push_calls_.push_back(PushCall{reader_id, count});
  }

  void unbind(std::uint64_t reader_id) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unbind_calls_.push_back(reader_id);
    if (owner_ == reader_id) {
      owner_ = 0;
    }
  }

  void stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stop_calls_;
  }

  std::uint64_t owner() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return owner_;
  }

  std::size_t bindAttempts() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return bind_calls_.size();
  }

  std::size_t pushCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return push_calls_.size();
  }

  BindCall firstBind() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return bind_calls_.front();
  }

private:
  mutable std::mutex mutex_;
  std::uint64_t owner_ = 0;
  std::vector<BindCall> bind_calls_;
  std::vector<PushCall> push_calls_;
  std::vector<std::uint64_t> unbind_calls_;
  std::size_t stop_calls_ = 0;
};

// Blocking fake stream: read() waits for a frame, EOS, close, or a scripted
// read failure. Tests push frames and assert the reader's resulting calls.
class FakeAudioOutputStream final : public AudioOutputStream
{
public:
  bool read(livekit::AudioFrameEvent & out_event) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    state_changed_.wait(lock, [this]() { return !queue_.empty() || eos_ || closed_ || throw_on_read_; });
    if (throw_on_read_) {
      throw_on_read_ = false;
      throw std::runtime_error("simulated read failure");
    }
    if (!queue_.empty()) {
      out_event = std::move(queue_.front());
      queue_.pop_front();
      return true;
    }
    return false;
  }

  void close() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    state_changed_.notify_all();
  }

  void pushFrame(int sample_rate, int channels, int samples_per_channel)
  {
    const std::size_t sample_count = static_cast<std::size_t>(channels) * static_cast<std::size_t>(samples_per_channel);
    livekit::AudioFrameEvent event;
    event.frame =
      livekit::AudioFrame(std::vector<std::int16_t>(sample_count, 0), sample_rate, channels, samples_per_channel);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(std::move(event));
    }
    state_changed_.notify_all();
  }

  void pushEos()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      eos_ = true;
    }
    state_changed_.notify_all();
  }

  void failNextRead()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      throw_on_read_ = true;
    }
    state_changed_.notify_all();
  }

  bool isClosed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable state_changed_;
  std::deque<livekit::AudioFrameEvent> queue_;
  bool eos_ = false;
  bool closed_ = false;
  bool throw_on_read_ = false;
};

// Owns the streams a test's manager creates, in creation order, so the test can
// push frames into the reader it wants to drive.
struct FakeStreamFactory
{
  std::vector<std::shared_ptr<FakeAudioOutputStream>> created;

  AudioOutputStreamFactory make()
  {
    return [this](const std::shared_ptr<livekit::Track> &, std::size_t) {
      auto stream = std::make_shared<FakeAudioOutputStream>();
      created.push_back(stream);
      return std::shared_ptr<AudioOutputStream>(stream);
    };
  }
};

class AudioOutputManagerTest : public test_support::RclcppTestSuite
{};

TEST_F(AudioOutputManagerTest, SubscribesToOutputTrackOnPublish)
{
  FakeRoomConnection connection;
  AudioOutputManager manager(connection, kTestSinkFragment);

  manager.onRemoteTrackPublished(
    RemoteTrackEvent{"participant-1", "PA_track1", kOutputTrackName, livekit::TrackKind::KIND_AUDIO, nullptr});

  ASSERT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.front().first, "participant-1");
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.front().second, "PA_track1");
}

TEST_F(AudioOutputManagerTest, IgnoresNonOutputTrackPublishes)
{
  FakeRoomConnection connection;
  AudioOutputManager manager(connection, kTestSinkFragment);

  manager.onRemoteTrackPublished(
    RemoteTrackEvent{
      "participant-1", "PA_track1", "lkros.audio.other.cab_mic", livekit::TrackKind::KIND_AUDIO, nullptr});
  manager.onRemoteTrackPublished(
    RemoteTrackEvent{"participant-1", "PA_track2", "some_video_feed", livekit::TrackKind::KIND_VIDEO, nullptr});

  EXPECT_TRUE(connection.state->subscribe_remote_track_calls.empty());
}

TEST_F(AudioOutputManagerTest, SubscribeFailureIsLoggedNotThrown)
{
  FakeRoomConnection connection;
  connection.setRemoteTrackSubscribable("participant-1", "PA_track1", false);
  AudioOutputManager manager(connection, kTestSinkFragment);

  EXPECT_NO_THROW(manager.onRemoteTrackPublished(
    RemoteTrackEvent{"participant-1", "PA_track1", kOutputTrackName, livekit::TrackKind::KIND_AUDIO, nullptr}));
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
}

TEST_F(AudioOutputManagerTest, FirstFrameBindsSinkWithFrameCaps)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  ASSERT_EQ(factory.created.size(), 1U);

  factory.created.front()->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->bindAttempts() >= 1U; }));

  const auto bind_call = sink->firstBind();
  EXPECT_EQ(bind_call.sample_rate, 48000);
  EXPECT_EQ(bind_call.channels, 1);
  EXPECT_EQ(sink->owner(), bind_call.reader_id);
}

TEST_F(AudioOutputManagerTest, FramesArePushedToTheOwningReader)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));

  factory.created.front()->pushFrame(48000, 1, 480);
  factory.created.front()->pushFrame(48000, 1, 480);

  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->pushCount() >= 2U; }));
  EXPECT_NE(sink->owner(), 0U);
}

TEST_F(AudioOutputManagerTest, ReaderRetriesUntilItOwnsTheSink)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track_a = connection.makeSyntheticRemoteTrack();
  auto track_b = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-a", track_a));
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-b", track_b));
  ASSERT_EQ(factory.created.size(), 2U);
  auto stream_a = factory.created[0];
  auto stream_b = factory.created[1];

  stream_a->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  const std::uint64_t owner_a = sink->owner();

  // B cannot claim while A is live; its first frame's bind must fail.
  stream_b->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->bindAttempts() >= 2U; }));
  EXPECT_EQ(sink->owner(), owner_a);

  // A leaves; only then may B's later frames retry and win the claim.
  manager.onRemoteTrackUnpublished(unpublishedOutputEvent("participant-a", track_a->sid()));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));

  stream_b->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  EXPECT_NE(sink->owner(), owner_a);
}

TEST_F(AudioOutputManagerTest, UnpublishClosesStreamAndReleasesSink)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  auto stream = factory.created.front();

  stream->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  EXPECT_FALSE(stream->isClosed());

  manager.onRemoteTrackUnpublished(unpublishedOutputEvent("participant-1", track->sid()));

  ASSERT_TRUE(test_support::waitUntil([&]() { return stream->isClosed(); }));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));
}

TEST_F(AudioOutputManagerTest, RemoteTrackSubscriptionFailureIsLoggedAndCleansUpStream)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  ASSERT_EQ(factory.created.size(), 1U);
  auto stream = factory.created.front();

  stream->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  EXPECT_FALSE(stream->isClosed());

  EXPECT_NO_THROW(manager.onRemoteTrackSubscriptionFailed(
    RemoteTrackSubscriptionFailedEvent{"participant-1", track->sid(), "denied"}));

  ASSERT_TRUE(test_support::waitUntil([&]() { return stream->isClosed(); }));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));
}

TEST_F(AudioOutputManagerTest, ParticipantDisconnectAndUnsubscribeAlsoCleanUp)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track1 = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track1));
  factory.created[0]->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  manager.onRemoteTrackUnsubscribed(subscribedOutputEvent("participant-1", track1));
  ASSERT_TRUE(test_support::waitUntil([&]() { return factory.created[0]->isClosed(); }));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));

  auto track2 = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-2", track2));
  ASSERT_EQ(factory.created.size(), 2U);
  factory.created[1]->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  auto disconnected_participant = makeDisconnectedParticipant("participant-2");
  manager.onParticipantDisconnected(makeDisconnectedEvent(disconnected_participant));
  ASSERT_TRUE(test_support::waitUntil([&]() { return factory.created[1]->isClosed(); }));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));
}

TEST_F(AudioOutputManagerTest, SecondOutputTrackDoesNotStealSinkWhileOwnerIsLive)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track_a = connection.makeSyntheticRemoteTrack();
  auto track_b = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-a", track_a));
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-b", track_b));

  // Reader A binds before it pushes, so wait for its push, not just its claim.
  factory.created[0]->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->pushCount() == 1U; }));
  const std::uint64_t owner_a = sink->owner();
  ASSERT_NE(owner_a, 0U);
  const std::size_t pushes_before = sink->pushCount();

  factory.created[1]->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->bindAttempts() >= 2U; }));

  EXPECT_EQ(sink->owner(), owner_a);
  EXPECT_EQ(sink->pushCount(), pushes_before);
}

TEST_F(AudioOutputManagerTest, ReadThrowIsCaughtAndUnbinds)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  auto manager = std::make_unique<AudioOutputManager>(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));

  factory.created.front()->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));

  factory.created.front()->failNextRead();

  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));
}

TEST_F(AudioOutputManagerTest, FailedReaderFreesItsSlotSoTheTrackCanResubscribe)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  auto manager = std::make_unique<AudioOutputManager>(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  ASSERT_EQ(factory.created.size(), 1U);

  factory.created.front()->failNextRead();

  // The reader closes its own stream once it has erased its slot.
  ASSERT_TRUE(test_support::waitUntil([&]() { return factory.created.front()->isClosed(); }));

  manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  ASSERT_EQ(factory.created.size(), 2U);

  factory.created.back()->pushFrame(48000, 1, 480);
  EXPECT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
}

TEST_F(AudioOutputManagerTest, EndOfStreamWithoutTrackEventFreesTheReaderSlot)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  auto manager = std::make_unique<AudioOutputManager>(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  ASSERT_EQ(factory.created.size(), 1U);

  factory.created.front()->pushEos();

  ASSERT_TRUE(test_support::waitUntil([&]() { return factory.created.front()->isClosed(); }));

  manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  EXPECT_EQ(factory.created.size(), 2U);
}

TEST_F(AudioOutputManagerTest, ConnectedResubscribesFromSnapshot)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  connection.setRemoteTrackSnapshot(
    {RoomConnection::RemoteTrackSnapshotEntry{
       "participant-1", "PA_track1", kOutputTrackName, livekit::TrackKind::KIND_AUDIO, false},
     RoomConnection::RemoteTrackSnapshotEntry{
       "participant-2", "PA_track2", "some_video_feed", livekit::TrackKind::KIND_VIDEO, false}});

  manager.onConnected();

  ASSERT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.front().first, "participant-1");
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.front().second, "PA_track1");
}

// An SDK resume reports Reconnecting then Reconnected and nothing else: the
// subscribed track and its media stay alive. The running reader must survive
// untouched, and Connected must not re-request the subscription.
TEST_F(AudioOutputManagerTest, ResumeKeepsTheRunningReader)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());
  routeRoomEvents(connection, manager);

  auto track = connection.makeSyntheticRemoteTrack(livekit::TrackKind::KIND_AUDIO, "PA_out");
  connection.setRemoteTrackSnapshot({RoomConnection::RemoteTrackSnapshotEntry{
    "publisher-1", "PA_out", kOutputTrackName, livekit::TrackKind::KIND_AUDIO, false}});
  connection.emitConnected();
  ASSERT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
  connection.emitRemoteTrackSubscribed("publisher-1", track, kOutputTrackName);
  ASSERT_EQ(factory.created.size(), 1U);
  auto stream = factory.created.front();

  stream->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->pushCount() >= 1U; }));
  const std::uint64_t owner = sink->owner();
  ASSERT_NE(owner, 0U);

  connection.emitReconnecting();
  connection.emitReconnected();

  EXPECT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
  EXPECT_EQ(factory.created.size(), 1U);
  EXPECT_FALSE(stream->isClosed());

  stream->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->pushCount() >= 2U; }));
  EXPECT_EQ(sink->owner(), owner);
  EXPECT_FALSE(stream->isClosed());
}

// A full restart unpublishes every remote track while still Connected, reports
// Reconnecting, re-announces the tracks while Reconnecting (not forwarded), and
// then reports Reconnected. The old reader must stop on the teardown events and
// Connected must subscribe the re-announced track, whose TrackSubscribed then
// starts a fresh reader under the same sid.
TEST_F(AudioOutputManagerTest, FullRestartResubscribesTheRepublishedTrack)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());
  routeRoomEvents(connection, manager);

  connection.emitConnected();
  connection.emitRemoteTrackPublished("publisher-1", "PA_out", kOutputTrackName);
  ASSERT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);
  auto old_track = connection.makeSyntheticRemoteTrack(livekit::TrackKind::KIND_AUDIO, "PA_out");
  connection.emitRemoteTrackSubscribed("publisher-1", old_track, kOutputTrackName);
  ASSERT_EQ(factory.created.size(), 1U);
  auto old_stream = factory.created.front();
  old_stream->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  const std::uint64_t old_owner = sink->owner();

  connection.emitRemoteTrackUnsubscribed("publisher-1", old_track, kOutputTrackName);
  connection.emitRemoteTrackUnpublished("publisher-1", "PA_out", kOutputTrackName);
  connection.emitParticipantDisconnected("publisher-1");
  ASSERT_TRUE(test_support::waitUntil([&]() { return old_stream->isClosed(); }));
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() == 0U; }));

  connection.emitReconnecting();
  connection.emitRemoteTrackPublished("publisher-1", "PA_out", kOutputTrackName);
  EXPECT_EQ(connection.state->subscribe_remote_track_calls.size(), 1U);

  connection.emitReconnected();
  ASSERT_EQ(connection.state->subscribe_remote_track_calls.size(), 2U);
  EXPECT_EQ(
    connection.state->subscribe_remote_track_calls.back(),
    (std::pair<std::string, std::string>{"publisher-1", "PA_out"}));

  auto new_track = connection.makeSyntheticRemoteTrack(livekit::TrackKind::KIND_AUDIO, "PA_out");
  connection.emitRemoteTrackSubscribed("publisher-1", new_track, kOutputTrackName);
  ASSERT_EQ(factory.created.size(), 2U);
  factory.created.back()->pushFrame(48000, 1, 480);
  ASSERT_TRUE(test_support::waitUntil([&]() { return sink->owner() != 0U; }));
  EXPECT_NE(sink->owner(), old_owner);
}

TEST_F(AudioOutputManagerTest, CleanupEventsAfterTheReaderIsGoneAreHarmless)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
  manager.onRemoteTrackUnpublished(unpublishedOutputEvent("participant-1", track->sid()));
  ASSERT_TRUE(factory.created.front()->isClosed());

  // Later cleanup events for the same track neither start a reader nor claim the sink.
  manager.onRemoteTrackUnsubscribed(subscribedOutputEvent("participant-1", track));
  auto disconnected_participant = makeDisconnectedParticipant("participant-1");
  manager.onParticipantDisconnected(makeDisconnectedEvent(disconnected_participant));
  EXPECT_EQ(factory.created.size(), 1U);
  EXPECT_EQ(sink->owner(), 0U);
}

TEST_F(AudioOutputManagerTest, ParticipantDisconnectIsSafeWithoutReaders)
{
  FakeRoomConnection connection;
  AudioOutputManager manager(connection, kTestSinkFragment);

  EXPECT_NO_THROW(connection.emitParticipantDisconnected("participant-1"));
  auto disconnected_participant = makeDisconnectedParticipant("participant-1");
  EXPECT_NO_THROW(manager.onParticipantDisconnected(makeDisconnectedEvent(disconnected_participant)));
}

TEST_F(AudioOutputManagerTest, NonOutputSubscribedTrackIsIgnored)
{
  FakeRoomConnection connection;
  auto sink = std::make_shared<FakeAudioOutputSink>();
  FakeStreamFactory factory;
  AudioOutputManager manager(connection, sink, factory.make());

  auto track = connection.makeSyntheticRemoteTrack();
  manager.onRemoteTrackSubscribed(
    RemoteTrackEvent{
      "participant-1", track->sid(), "lkros.audio.other.cab_mic", livekit::TrackKind::KIND_AUDIO, track});

  EXPECT_TRUE(factory.created.empty());
}

// Stress the destructor's reader-drain wait: a regression in the wait/notify
// bookkeeping manifests as a hang that trips the test timeout, not as a failed
// expectation. Each iteration starts a live reader and waits for it to exit
// during destruction, maximizing exposure to a lost wakeup.
TEST_F(AudioOutputManagerTest, ReaderShutdownWaitDoesNotLoseWakeups)
{
  for (int iteration = 0; iteration < 300; ++iteration) {
    FakeRoomConnection connection;
    auto sink = std::make_shared<FakeAudioOutputSink>();
    FakeStreamFactory factory;
    AudioOutputManager manager(connection, sink, factory.make());

    auto track = connection.makeSyntheticRemoteTrack();
    manager.onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
    factory.created.front()->pushFrame(48000, 1, 480);

    // The manager and this iteration's reader drain at the closing brace.
  }
}

// A reader must drop its stream and sink references before it releases the
// destructor; otherwise the last reference to a LiveKit AudioStream could be
// destroyed on the detached thread after the manager (and possibly the SDK) is
// gone. Once the manager is destroyed, the test must hold the only references.
// A regression shows up as a timing-dependent failure, so iterate.
TEST_F(AudioOutputManagerTest, ReaderReleasesStreamAndSinkBeforeDestructionReturns)
{
  for (int iteration = 0; iteration < 100; ++iteration) {
    FakeRoomConnection connection;
    auto sink = std::make_shared<FakeAudioOutputSink>();
    FakeStreamFactory factory;
    auto manager = std::make_unique<AudioOutputManager>(connection, sink, factory.make());

    auto track = connection.makeSyntheticRemoteTrack();
    manager->onRemoteTrackSubscribed(subscribedOutputEvent("participant-1", track));
    factory.created.front()->pushFrame(48000, 1, 480);

    manager.reset();

    EXPECT_EQ(factory.created.front().use_count(), 1);
    EXPECT_EQ(sink.use_count(), 1);
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge::audio
