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

#include "audio/audio_output_sink.hpp"

#include <gst/app/gstappsrc.h>
#include <gst/base/gstbasesink.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.audio_out_sink");
constexpr auto kRestartDelay = std::chrono::milliseconds(250);
constexpr auto kRestartFailureLogThrottle = std::chrono::seconds(5);

void disableSyncIfSink(GstElement * element)
{
  if (GST_IS_BASE_SINK(element)) {
    gst_base_sink_set_sync(GST_BASE_SINK(element), FALSE);
  }
}

void onDeepElementAdded(GstBin *, GstBin *, GstElement * element, gpointer)
{
  disableSyncIfSink(element);
}

}  // namespace

// Receive tail: the bridge owns the edge and the output device's own buffering
// paces playback, so the configured fragment is used verbatim after the
// convert/resample stages, with sink sync off (see disableAudioOutputSinkSync).
// appsrc is the only buffer: past kAudioOutputMaxBacklog it drops the oldest
// audio, so a slow output costs a skip rather than a growing delay.
std::string buildAudioOutputSinkPipelineDescription(const std::string & sink_fragment)
{
  std::string description = "appsrc name=";
  description += kBridgeAppSrcName;
  description += " is-live=true block=false format=time do-timestamp=false";
  description += " max-bytes=0 max-buffers=0 leaky-type=downstream max-time=";
  description += std::to_string(kAudioOutputMaxBacklog);
  description += " ! audioconvert";
  description += " ! audioresample";
  description += " ! ";
  description += sink_fragment;
  return description;
}

AudioOutputBufferTiming computeAudioOutputBufferTiming(
  std::size_t sample_count, int channels, int sample_rate, GstClockTime next_pts)
{
  const int effective_channels = channels > 0 ? channels : 1;
  const int effective_rate = sample_rate > 0 ? sample_rate : 48000;
  const std::size_t frame_count = sample_count / static_cast<std::size_t>(effective_channels);
  const GstClockTime duration =
    static_cast<GstClockTime>(frame_count) * GST_SECOND / static_cast<GstClockTime>(effective_rate);
  return {next_pts, duration};
}

void disableAudioOutputSinkSync(GstElement * pipeline)
{
  // Catches sinks that bins like autoaudiosink create on a later state change.
  g_signal_connect(pipeline, "deep-element-added", G_CALLBACK(onDeepElementAdded), nullptr);

  utils::GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline)));
  utils::GValueSlot item;
  while (true) {
    const GstIteratorResult result = gst_iterator_next(iterator.get(), item.get());
    if (result == GST_ITERATOR_DONE) {
      return;
    }
    if (result == GST_ITERATOR_RESYNC) {
      gst_iterator_resync(iterator.get());
      continue;
    }
    if (result != GST_ITERATOR_OK) {
      throw std::runtime_error("Could not inspect the audio output sink pipeline's elements.");
    }

    disableSyncIfSink(GST_ELEMENT(g_value_get_object(item.get())));
    item.reset();
  }
}

AudioOutputSink::AudioOutputSink(std::string sink_fragment)
: sink_fragment_(std::move(sink_fragment))
, failure_handler_(kRestartDelay, [this]() { restartPipeline(); })
{}

AudioOutputSink::~AudioOutputSink()
{
  stop();
}

bool AudioOutputSink::bind(std::uint64_t reader_id, int sample_rate, int num_channels)
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return false;
  }

  std::uint64_t expected = 0;
  if (!owner_.compare_exchange_strong(expected, reader_id)) {
    logIgnoredOnce(reader_id);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A rebind (lease handover) may arrive while the previous pipeline is still
    // PLAYING; stop it first so two pipelines never compete for the device.
    stopPipelineLocked();
    caps_rate_ = sample_rate;
    caps_channels_ = num_channels;
    try {
      startPipelineLocked();
    } catch (const std::exception & exception) {
      // A sink that is dead at bind time is not a binding failure: keep the claim so the owning
      // reader's live frame cadence re-arms the restart loop (push() schedules while the pipeline
      // is down) and playback self-heals when the device returns. Ownership is released only by
      // unbind()/reader finalize or stop().
      LogEvent(kLogger, "audio_out_sink_start_failed").fieldOr("error", exception.what()).warn();
      stopPipelineLocked();
    }
  }

  return true;
}

void AudioOutputSink::push(std::uint64_t reader_id, const std::int16_t * samples, std::size_t count)
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (owner_.load(std::memory_order_acquire) != reader_id) {
    logIgnoredOnce(reader_id);
    return;
  }
  if (samples == nullptr || count == 0) {
    return;
  }
  // While the pipeline is down, each live frame re-arms the restart loop, which
  // cannot re-arm itself (a failed restart's own bus error is coalesced). Idle
  // bridges with no frames never cycle the device.
  if (!pipeline_active_.load(std::memory_order_acquire)) {
    (void)failure_handler_.schedule();
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (pipeline_ == nullptr || appsrc_element_ == nullptr) {
    return;
  }

  const std::size_t byte_size = count * sizeof(std::int16_t);
  utils::GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, byte_size, nullptr));
  if (buffer == nullptr) {
    LogEvent(kLogger, "audio_out_sink_push_failed").field("reason", "buffer_alloc_failed").warn();
    return;
  }

  {
    utils::GstBufferMap mapping(*buffer, GST_MAP_WRITE);
    if (!mapping.is_valid()) {
      LogEvent(kLogger, "audio_out_sink_push_failed").field("reason", "buffer_map_failed").warn();
      return;
    }
    std::memcpy(mapping.get()->data, samples, byte_size);
  }

  // Explicit frame-count PTS/DURATION (do-timestamp=false): the audio clock
  // defines time as a perfectly regular stamp train, instead of wall-clock
  // arrival stamps that jitter against the pipeline clock and crackle.
  const AudioOutputBufferTiming timing = computeAudioOutputBufferTiming(count, caps_channels_, caps_rate_, next_pts_);
  next_pts_ += timing.duration;

  GST_BUFFER_PTS(buffer.get()) = timing.pts;
  GST_BUFFER_DTS(buffer.get()) = timing.pts;
  GST_BUFFER_DURATION(buffer.get()) = timing.duration;

  // Failure must not tear anything down — appsrc is block=false, so the next
  // push simply reclaims the pipeline.
  const GstFlowReturn result = gst_app_src_push_buffer(GST_APP_SRC(appsrc_element_.get()), buffer.release());
  if (result != GST_FLOW_OK) {
    LogEvent(kLogger, "audio_out_sink_push_failed")
      .field("reason", "push_return")
      .field("flow_return", static_cast<int>(result))
      .warn();
  }
}

void AudioOutputSink::unbind(std::uint64_t reader_id)
{
  // Release the claim under mutex_: a new owner's bind() then waits for this
  // stop, so it cannot start its pipeline before this reader tears one down.
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint64_t expected = reader_id;
  if (!owner_.compare_exchange_strong(expected, 0)) {
    return;
  }

  // Releasing the claim must also release the output device: otherwise a
  // coalesced restart could reopen the device after the output track
  // unpublishes. Cancel the queued restart so it cannot run at all;
  // restartPipeline()'s owner_ == 0 guard is the second line of defence.
  stopPipelineLocked();
  failure_handler_.cancelPending();
}

void AudioOutputSink::stop()
{
  is_shutdown_.store(true, std::memory_order_release);
  failure_handler_.close();

  std::lock_guard<std::mutex> lock(mutex_);
  stopPipelineLocked();
}

bool AudioOutputSink::hasActivePipeline() const
{
  return pipeline_active_.load(std::memory_order_acquire);
}

std::size_t AudioOutputSink::pipelineStartAttempts() const
{
  return pipeline_start_attempts_.load(std::memory_order_relaxed);
}

// This path must stay lock-free against mutex_: the sync bus handler can
// deliver a failure from inside startPipelineLocked() or restartPipeline(),
// which hold mutex_ while GStreamer performs the state change. Locking here
// would deadlock against the calling thread itself. schedule() coalesces
// duplicate failures, and close() marks the handler closed before teardown, so
// no sink mutex_ is needed.
void AudioOutputSink::onBusMessage(GstMessage * message)
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (message == nullptr || GST_MESSAGE_TYPE(message) != GST_MESSAGE_ERROR) {
    return;
  }

  GError * raw_error = nullptr;
  gst_message_parse_error(message, &raw_error, nullptr);
  utils::GErrorPtr error(raw_error);
  const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";

  if (!failure_handler_.schedule()) {
    return;
  }

  LogEvent(kLogger, "audio_out_sink_restart_scheduled")
    .fieldOr("reason", reason)
    .field("restart_delay_ms", kRestartDelay.count())
    .warn();
}

void AudioOutputSink::restartPipeline()
{
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  // A restart scheduled before an unbind must not reopen the device: no
  // pipeline may run without an owner to feed it.
  if (owner_.load(std::memory_order_acquire) == 0) {
    return;
  }

  stopPipelineLocked();
  try {
    startPipelineLocked();
  } catch (const std::exception & exception) {
    // No retry cap: a permanently missing device restarts at ~4/s, bounded by
    // the 250 ms delay, while audio keeps arriving. Idle robots never restart:
    // re-arms come only from live frames on the track.
    LogEvent(kLogger, "audio_out_sink_restart_failed")
      .fieldOr("error", exception.what())
      .warnThrottle(log_clock_, kRestartFailureLogThrottle);
  }
}

void AudioOutputSink::startPipelineLocked()
{
  (void)pipeline_start_attempts_.fetch_add(1, std::memory_order_relaxed);

  utils::ensureGStreamerInitialized();

  if (sink_fragment_.empty()) {
    throw std::runtime_error("Audio output sink fragment is not configured.");
  }
  if (caps_rate_ <= 0 || caps_channels_ <= 0) {
    throw std::runtime_error("Audio output sink caps are not set.");
  }

  utils::GstElementPtr pipeline(
    gst_parse_launch(buildAudioOutputSinkPipelineDescription(sink_fragment_).c_str(), nullptr));
  if (pipeline == nullptr) {
    throw std::runtime_error("Failed to create GStreamer audio output sink pipeline.");
  }

  utils::GstElementPtr appsrc_element(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeAppSrcName));
  if (appsrc_element == nullptr || !GST_IS_APP_SRC(appsrc_element.get())) {
    throw std::runtime_error("Audio output sink pipeline did not create the expected appsrc.");
  }

  std::string caps_string = "audio/x-raw,format=S16LE,layout=interleaved,rate=";
  caps_string += std::to_string(caps_rate_);
  caps_string += ",channels=";
  caps_string += std::to_string(caps_channels_);

  GstCaps * raw_caps = gst_caps_from_string(caps_string.c_str());
  if (raw_caps == nullptr) {
    throw std::runtime_error("Failed to parse audio output sink caps: " + caps_string);
  }
  // gst_caps_from_string returns a full reference to a GstMiniObject, released
  // with gst_caps_unref via GstCapsPtr. set_caps takes its own reference.
  utils::GstCapsPtr caps(raw_caps);

  gst_app_src_set_caps(GST_APP_SRC(appsrc_element.get()), caps.get());
  gst_app_src_set_stream_type(GST_APP_SRC(appsrc_element.get()), GST_APP_STREAM_TYPE_STREAM);
  disableAudioOutputSinkSync(pipeline.get());

  utils::GstBusPtr bus(gst_element_get_bus(pipeline.get()));
  gst_bus_set_sync_handler(
    bus.get(),
    [](GstBus *, GstMessage * message, gpointer user_data) -> GstBusSyncReply {
      static_cast<AudioOutputSink *>(user_data)->onBusMessage(message);
      return GST_BUS_PASS;
    },
    this,
    nullptr);

  pipeline_ = std::move(pipeline);
  // appsrc_element_ owns the reference gst_bin_get_by_name returned and is
  // released before pipeline_ in stopPipelineLocked().
  appsrc_element_ = std::move(appsrc_element);
  next_pts_ = 0;  // fresh pipeline = fresh clock base
  pipeline_active_.store(true, std::memory_order_release);

  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (result == GST_STATE_CHANGE_FAILURE) {
    stopPipelineLocked();
    throw std::runtime_error("Failed to set audio output sink pipeline to PLAYING.");
  }

  LogEvent(kLogger, "audio_out_sink_started").field("caps", caps_string).info();
}

void AudioOutputSink::stopPipelineLocked()
{
  if (pipeline_ == nullptr) {
    return;
  }

  utils::GstBusPtr bus(gst_element_get_bus(pipeline_.get()));
  gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);

  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  if (result == GST_STATE_CHANGE_ASYNC) {
    (void)gst_element_get_state(pipeline_.get(), nullptr, nullptr, GST_CLOCK_TIME_NONE);
  }

  appsrc_element_.reset();
  pipeline_.reset();
  pipeline_active_.store(false, std::memory_order_release);
}

void AudioOutputSink::logIgnoredOnce(std::uint64_t reader_id)
{
  // Only log when the dropped-frame source changes. Bounded state: a single
  // sentinel-guarded word instead of a set that grows per reader.
  if (last_ignored_reader_.exchange(reader_id, std::memory_order_acq_rel) == reader_id) {
    return;
  }
  LogEvent(kLogger, "audio_out_sink_frame_dropped")
    .field("reader_id", reader_id)
    .field("owner_id", owner_.load(std::memory_order_acquire))
    .debug();
}

}  // namespace livekit_ros2_bridge::audio
