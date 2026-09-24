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

#include "audio/audio_output_manager.hpp"

#include <condition_variable>
#include <exception>
#include <thread>
#include <utility>
#include <vector>

#include "livekit/participant.h"
#include "livekit/remote_participant.h"
#include "protocol/constants.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.audio_out");
// Ring-buffer capacity for AudioStream — newest-wins so a stalled reader drains
// stale audio instead of lagging unboundedly on a lossy link.
constexpr std::size_t kStreamCapacity = 50;

// Thin adapter from the test-facing AudioOutputStream seam onto the SDK's
// livekit::AudioStream, the production stream the reader consumes.
class LiveKitAudioOutputStream final : public AudioOutputStream
{
public:
  explicit LiveKitAudioOutputStream(std::shared_ptr<livekit::AudioStream> stream)
  : stream_(std::move(stream))
  {}

  bool read(livekit::AudioFrameEvent & out_event) override
  {
    return stream_->read(out_event);
  }

  void close() override
  {
    stream_->close();
  }

private:
  std::shared_ptr<livekit::AudioStream> stream_;
};

std::shared_ptr<AudioOutputStream> makeLiveKitAudioOutputStream(
  const std::shared_ptr<livekit::Track> & track, std::size_t capacity)
{
  livekit::AudioStream::Options options;
  options.capacity = capacity;
  return std::make_shared<LiveKitAudioOutputStream>(livekit::AudioStream::fromTrack(track, options));
}

}  // namespace

AudioOutputManager::AudioOutputManager(RoomConnection & room_connection, std::string sink_fragment)
: AudioOutputManager(
    room_connection, std::make_shared<AudioOutputSink>(std::move(sink_fragment)), makeLiveKitAudioOutputStream)
{}

AudioOutputManager::AudioOutputManager(
  RoomConnection & room_connection,
  std::shared_ptr<AudioOutputSinkInterface> sink,
  AudioOutputStreamFactory stream_factory)
: room_connection_(room_connection)
, sink_(std::move(sink))
, stream_factory_(std::move(stream_factory))
{}

AudioOutputManager::~AudioOutputManager()
{
  std::map<std::string, std::shared_ptr<Reader>> readers;
  {
    // Take event_mutex_ so no in-flight handler can add a reader after this
    // snapshot; then release it before blocking on any reader.
    std::lock_guard<std::mutex> event_lock(event_mutex_);
    is_shutdown_.store(true, std::memory_order_release);
    std::lock_guard<std::mutex> lock(mutex_);
    readers = std::move(readers_);
    readers_.clear();
  }

  // Signal every reader and close its stream so a read() blocked between
  // frames wakes and exits.
  for (auto & [track_sid, reader] : readers) {
    (void)track_sid;
    reader->stop.store(true, std::memory_order_release);
    if (reader->stream != nullptr) {
      reader->stream->close();
    }
  }

  sink_->stop();

  // Wait for every detached reader to finish its ScopeExit. live_readers_ is
  // incremented before each spawn and, as each reader's last act (after it
  // drops its stream and sink references), decremented and notified under
  // wait_mutex_. Once it reaches zero no thread can still be inside LiveKit FFI
  // or touch this manager again.
  std::unique_lock<std::mutex> wait_lock(wait_mutex_);
  reader_exited_.wait(wait_lock, [this]() { return live_readers_.load(std::memory_order_acquire) == 0; });
}

void AudioOutputManager::onRemoteTrackPublished(const RemoteTrackEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  if (event.track_name != protocol::kAudioOutTrackName) {
    return;
  }

  // The publisher's identity reaches logs here; the bridge performs no
  // identity checks on the audio output track. Logged only for that
  // track so busy rooms do not emit an info line per foreign publication.
  LogEvent(kLogger, "remote_track_published")
    .fieldOr("participant_identity", event.participant_identity)
    .fieldOr("track_sid", event.track_sid)
    .fieldQuoted("track_name", event.track_name)
    .info();

  // A second live output track still gets subscribed (it is the named
  // track); its frames are then logged once and dropped by the single active
  // sink, so it can never steal the sink from the active track.
  if (!room_connection_.subscribeRemoteTrack(event.participant_identity, event.track_sid)) {
    LogEvent(kLogger, "audio_out_subscribe_failed")
      .fieldOr("participant_identity", event.participant_identity)
      .fieldOr("track_sid", event.track_sid)
      .warn();
  }
}

void AudioOutputManager::onRemoteTrackUnpublished(const RemoteTrackEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  if (event.track_name != protocol::kAudioOutTrackName) {
    return;
  }

  LogEvent(kLogger, "remote_track_unpublished")
    .fieldOr("participant_identity", event.participant_identity)
    .fieldOr("track_sid", event.track_sid)
    .fieldQuoted("track_name", event.track_name)
    .info();

  stopReader(event.track_sid, "track_unpublished");
}

void AudioOutputManager::onRemoteTrackSubscribed(const RemoteTrackEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (
    event.track == nullptr || event.track_kind != livekit::TrackKind::KIND_AUDIO ||
    event.track_name != protocol::kAudioOutTrackName)
  {
    LogEvent(kLogger, "audio_out_track_ignored")
      .field("reason", "not_audio_out")
      .fieldOr("track_sid", event.track_sid)
      .fieldQuoted("track_name", event.track_name)
      .debug();
    return;
  }

  subscribeOutputTrack(event);
}

void AudioOutputManager::onRemoteTrackUnsubscribed(const RemoteTrackEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (event.track_name == protocol::kAudioOutTrackName) {
    stopReader(event.track_sid, "track_unsubscribed");
  }
}

void AudioOutputManager::onRemoteTrackSubscriptionFailed(const RemoteTrackSubscriptionFailedEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  LogEvent(kLogger, "audio_out_subscription_failed")
    .fieldOr("participant_identity", event.participant_identity)
    .fieldOr("track_sid", event.track_sid)
    .fieldOr("error", event.error)
    .warn();

  stopReader(event.track_sid, "subscription_failed");
}

void AudioOutputManager::onParticipantDisconnected(const livekit::ParticipantDisconnectedEvent & event)
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  const auto * participant = event.participant;
  if (participant == nullptr) {
    return;
  }

  // Snapshot the reader keys owned by this identity, then stop them outside
  // mutex_ (stopReader() also takes it).
  std::vector<std::string> track_sids_to_stop;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [track_sid, reader] : readers_) {
      if (reader->participant_identity == participant->identity()) {
        track_sids_to_stop.push_back(track_sid);
      }
    }
  }

  for (const std::string & track_sid : track_sids_to_stop) {
    stopReader(track_sid, "participant_disconnected");
  }
}

void AudioOutputManager::onConnected()
{
  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  // Readers are deliberately not stopped here. After a resume the SDK keeps
  // the subscribed track and its stream alive and sends no track events, so a
  // stopped reader would never be recreated. After a full restart the old
  // readers already ended on the unsubscribe/unpublish events the SDK sent
  // before Reconnecting, and the re-announced track is picked up below.
  snapshotSubscribe();
}

void AudioOutputManager::subscribeOutputTrack(const RemoteTrackEvent & event)
{
  if (event.track == nullptr || event.track_sid.empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_.load(std::memory_order_acquire) || readers_.find(event.track_sid) != readers_.end()) {
      return;
    }
  }

  std::shared_ptr<AudioOutputStream> stream;
  try {
    stream = stream_factory_(event.track, kStreamCapacity);
  } catch (...) {
    LogEvent(kLogger, "audio_out_stream_create_failed")
      .fieldOr("track_sid", event.track_sid)
      .fieldException("error", std::current_exception())
      .warn();
    return;
  }

  auto reader = std::make_shared<Reader>();
  reader->participant_identity = event.participant_identity;
  reader->track_sid = event.track_sid;
  reader->stream = stream;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_.load(std::memory_order_acquire)) {
      return;
    }
    readers_[event.track_sid] = reader;
  }

  const std::uint64_t reader_id = last_reader_id_.fetch_add(1, std::memory_order_relaxed) + 1;

  LogEvent(kLogger, "audio_out_reader_started")
    .field("reader_id", reader_id)
    .fieldOr("track_sid", event.track_sid)
    .fieldQuoted("track_name", event.track_name)
    .fieldOr("participant_identity", event.participant_identity)
    .info();

  live_readers_.fetch_add(1, std::memory_order_acq_rel);

  // Dedicated reader thread: reads decoded PCM and feeds the playback sink.
  // It captures `this` for the live-reader counter, which is safe because the
  // destructor waits for live_readers_ to reach zero before returning and the
  // thread touches no member after its final decrement. The reader owns the
  // sink claim once it binds and releases it in the ScopeExit, so a handover
  // rebinds the sink with zero bridge-side identity knowledge.
  try {
    std::thread([this, reader, stream, reader_id, sink = sink_]() mutable {
      const std::string track_sid = reader->track_sid;

      std::uint64_t total_frames = 0;
      bool first_frame_logged = false;
      bool sink_owned = false;

      // Runs on every exit path: free the slot, release the sink, drop
      // references, then decrement the live-reader count.
      ScopeExit on_reader_exit([this, &reader, &stream, &sink, &track_sid, reader_id, &total_frames]() {
        // Ended on its own: free the slot so a later subscribe is not ignored.
        if (!reader->stop.load(std::memory_order_acquire)) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto entry = readers_.find(track_sid);
            if (entry != readers_.end() && entry->second == reader) {
              readers_.erase(entry);
            }
          }
          stream->close();
        }
        sink->unbind(reader_id);
        LogEvent(kLogger, "audio_out_reader_stopped")
          .field("reader_id", reader_id)
          .fieldOr("track_sid", track_sid)
          .field("frames_received", total_frames)
          .info();
        // Release this thread's references while the manager is still alive.
        // Once the count reaches zero the destructor may return and the SDK may
        // shut down, so this detached thread must not be the one to destroy
        // the last AudioStream or sink reference after that point.
        stream.reset();
        reader.reset();
        sink.reset();
        // Decrement and notify under wait_mutex_: the destructor can neither
        // slip between its predicate check and its wait and miss the wakeup,
        // nor observe zero and free reader_exited_ before notify_all() returns.
        // Nothing after the unlock may touch a member.
        std::lock_guard<std::mutex> exit_lock(wait_mutex_);
        live_readers_.fetch_sub(1, std::memory_order_acq_rel);
        reader_exited_.notify_all();
      });

      // One backstop for the whole body: read()/bind()/push() are SDK/GStreamer
      // calls that may throw across the FFI boundary. An exception escaping a
      // detached thread calls std::terminate(), so nothing is allowed past this
      // boundary; every failure is logged, then the ScopeExit finalizes.
      try {
        livekit::AudioFrameEvent frame_event;
        while (!reader->stop.load(std::memory_order_acquire)) {
          bool got_frame = false;
          try {
            got_frame = stream->read(frame_event);
          } catch (...) {
            LogEvent(kLogger, "audio_out_read_failed")
              .field("reader_id", reader_id)
              .fieldOr("track_sid", track_sid)
              .fieldException("error", std::current_exception())
              .error();
            break;
          }

          if (!got_frame) {
            // EOS / close(): track gone, SDK disconnect, or stream closed.
            break;
          }

          const auto & frame = frame_event.frame;
          const auto & samples = frame.data();
          if (samples.empty()) {
            continue;
          }

          // Retry the claim on every frame until it succeeds. A handover can
          // find the previous owner not yet finalized, so a one-shot bind on
          // the first frame would permanently silence this reader; once owned,
          // stop retrying. Caps are taken from the claiming frame's actual
          // rate/channels. Mute is silence-through: silent frames keep flowing
          // so the sink's claim never blocks the next holder.
          if (!sink_owned) {
            sink_owned = sink->bind(reader_id, frame.sampleRate(), frame.numChannels());
          }

          if (!first_frame_logged) {
            first_frame_logged = true;
            LogEvent(kLogger, "audio_out_first_frame")
              .field("reader_id", reader_id)
              .fieldOr("track_sid", track_sid)
              .field("sample_rate", frame.sampleRate())
              .field("channels", frame.numChannels())
              .field("samples_per_channel", frame.samplesPerChannel())
              .field("sink_bound", sink_owned)
              .info();
          }

          // While unbound, push() logs once and drops; live frames still re-arm
          // the sink's rate-bounded restart loop, so a restored device self-heals.
          sink->push(reader_id, samples.data(), samples.size());
          ++total_frames;
        }
      } catch (...) {
        LogEvent(kLogger, "audio_out_reader_failed")
          .field("reader_id", reader_id)
          .fieldOr("track_sid", track_sid)
          .fieldException("error", std::current_exception())
          .error();
      }
    }).detach();
  } catch (...) {
    {
      // Notify under wait_mutex_ to match the reader's exit path. No destructor
      // can be waiting here: this runs under event_mutex_, which the destructor
      // takes before it waits.
      std::lock_guard<std::mutex> exit_lock(wait_mutex_);
      live_readers_.fetch_sub(1, std::memory_order_acq_rel);
      reader_exited_.notify_all();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      readers_.erase(event.track_sid);
    }
    LogEvent(kLogger, "audio_out_reader_start_failed")
      .fieldOr("track_sid", event.track_sid)
      .fieldException("error", std::current_exception())
      .warn();
    return;
  }
}

void AudioOutputManager::stopReader(const std::string & track_sid, const char * reason)
{
  std::shared_ptr<Reader> reader;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto entry = readers_.find(track_sid);
    if (entry == readers_.end()) {
      return;
    }
    reader = entry->second;
    readers_.erase(entry);
  }

  reader->stop.store(true, std::memory_order_release);
  if (reader->stream != nullptr) {
    reader->stream->close();
  }
  LogEvent(kLogger, "audio_out_reader_stopping").fieldOr("track_sid", track_sid).field("reason", reason).info();
}

void AudioOutputManager::snapshotSubscribe()
{
  const auto snapshot = room_connection_.remoteTrackSnapshot();
  for (const auto & entry : snapshot) {
    if (entry.track_name == protocol::kAudioOutTrackName && !entry.subscribed) {
      (void)room_connection_.subscribeRemoteTrack(entry.participant_identity, entry.track_sid);
    }
  }
}

}  // namespace livekit_ros2_bridge::audio
