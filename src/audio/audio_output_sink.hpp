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

#include <gst/gst.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

#include "rclcpp/clock.hpp"
#include "utils/gstreamer_resources.hpp"
#include "utils/pipeline_failure_handler.hpp"

// The appsrc element is held as a plain GstElementPtr; appsrc call sites cast
// with GST_APP_SRC, so this header does not force gst-app includes on all
// transitive consumers.

namespace livekit_ros2_bridge::audio
{

// Wire name of the bridge-owned appsrc in the audio output playback pipeline. The
// startup validator reserves it so a sink fragment cannot define its own
// endpoint.
inline constexpr char kBridgeAppSrcName[] = "bridge_audio_out_src";

// Most audio appsrc holds before dropping the oldest, bounding added delay.
inline constexpr GstClockTime kAudioOutputMaxBacklog = 200 * GST_MSECOND;

// Builds the playback pipeline description: the bridge-owned appsrc, capped at
// kAudioOutputMaxBacklog, then audioconvert, audioresample, and the verbatim
// sink fragment. Shared with startup validation so the validated pipeline
// matches the one that runs.
std::string buildAudioOutputSinkPipelineDescription(const std::string & sink_fragment);

// Timing for one interleaved S16 buffer. The per-channel frame count drives the
// duration, so a stereo buffer advances the playback clock the same wall time as
// a mono buffer with the same number of frames. The caller threads next_pts
// through push(); keeping a running GstClockTime avoids the long-run overflow of
// multiplying an ever-growing sample counter by GST_SECOND.
struct AudioOutputBufferTiming
{
  GstClockTime pts;
  GstClockTime duration;
};

// Computes PTS/DURATION for one interleaved S16 buffer. A non-positive channel
// count is treated as mono; a non-positive rate falls back to 48000 Hz.
AudioOutputBufferTiming computeAudioOutputBufferTiming(
  std::size_t sample_count, int channels, int sample_rate, GstClockTime next_pts);

// Turns off sync on every sink, including ones added later. A synced sink whose
// device delay exceeds its declared latency plays silence without an error.
void disableAudioOutputSinkSync(GstElement * pipeline);

// Abstract playback edge used by AudioOutputManager so its reader-handover and
// shutdown logic can be driven with a fake sink in tests. The concrete
// AudioOutputSink below is the production implementation.
class AudioOutputSinkInterface
{
public:
  virtual ~AudioOutputSinkInterface() = default;
  virtual bool bind(std::uint64_t reader_id, int sample_rate, int num_channels) = 0;
  virtual void push(std::uint64_t reader_id, const std::int16_t * samples, std::size_t count) = 0;
  virtual void unbind(std::uint64_t reader_id) = 0;
  virtual void stop() = 0;
};

// Plays received audio output PCM through appsrc (capped, drops oldest) →
// audioconvert → audioresample → the configured sink fragment. The
// pipeline is created lazily because appsrc caps come from the first frame's
// actual rate/channels.
//
// Ownership: refcounted; each AudioOutputManager reader thread captures it by
// copy, so the sink may outlive a single reader. ~AudioOutputSink stops the
// pipeline and closes the failure handler.
//
// Concurrency: frames arrive on per-track reader threads, failures arrive on
// GStreamer bus threads. A single reader owns the sink at a time (atomic CAS on
// a reader id); frames from other readers are logged once and dropped, so a
// second live output track can never steal the sink from the active one.
class AudioOutputSink : public AudioOutputSinkInterface
{
public:
  explicit AudioOutputSink(std::string sink_fragment);
  ~AudioOutputSink() override;

  AudioOutputSink(const AudioOutputSink &) = delete;
  AudioOutputSink & operator=(const AudioOutputSink &) = delete;
  AudioOutputSink(AudioOutputSink &&) = delete;
  AudioOutputSink & operator=(AudioOutputSink &&) = delete;

  // Claims the sink for this reader (first caller wins) and lazily starts the
  // playback pipeline with caps built from this frame. Only the owning reader's
  // first frame ever starts the pipeline. Returns true when this reader owns the
  // sink; a failed initial pipeline start still reports the claim, because the
  // owning reader's live frame cadence drives the restart loop until the device
  // returns.
  bool bind(std::uint64_t reader_id, int sample_rate, int num_channels) override;

  // Pushes one interleaved S16 frame. Non-owner frames are logged once and
  // dropped. Push failures are logged and dropped — never tear down (appsrc is
  // block=false). While the pipeline is down, the caller's live frame cadence
  // re-arms the rate-bounded restart loop; nothing restarts while no frames
  // arrive.
  void push(std::uint64_t reader_id, const std::int16_t * samples, std::size_t count) override;

  // Releases the claim on reader finalize so the next output track can claim
  // on its first frame (lease-handover rebind, with no bridge-side identity
  // knowledge). No-op when this reader did not own the sink.
  void unbind(std::uint64_t reader_id) override;

  // Stops the pipeline and disables restarts. Idempotent.
  void stop() override;

  // True from just before a pipeline is set to PLAYING until it is stopped.
  // Lock-free, so callers can observe lifecycle without contending on mutex_.
  bool hasActivePipeline() const;

  // Number of startPipelineLocked() invocations. The restart loop is
  // rate-bounded, so tests assert growth rather than absolute counts.
  std::size_t pipelineStartAttempts() const;

private:
  void startPipelineLocked();
  void stopPipelineLocked();
  void restartPipeline();
  void onBusMessage(GstMessage * message);
  void logIgnoredOnce(std::uint64_t reader_id);

  std::string sink_fragment_;

  // Guards pipeline_/appsrc_/caps state. The failure path (onBusMessage →
  // schedule) must stay lock-free against this mutex: the sync bus handler can
  // fire from inside startPipelineLocked() while a caller holds it.
  std::mutex mutex_;
  // Throttles the ~4/s restart-failure log while the device is gone.
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  utils::GstElementPtr pipeline_;
  utils::GstElementPtr appsrc_element_;
  int caps_rate_ = 0;
  int caps_channels_ = 0;

  // PTS of the next buffer on the current pipeline instance; the running source
  // of buffer PTS/DURATION (audio clock, not wall clock). Reset in
  // startPipelineLocked() so a restarted pipeline sees timestamps from 0. A
  // running GstClockTime avoids the long-run overflow of an ever-growing
  // sample counter multiplied by GST_SECOND.
  GstClockTime next_pts_ = 0;

  // Bumped at the top of every startPipelineLocked() so tests can tell idle
  // sinks (no frames, no restarts) from sinks whose live frame cadence re-arms
  // the restart loop.
  std::atomic<std::size_t> pipeline_start_attempts_{0};

  // Lock-free mirror of pipeline_ != nullptr so push() can re-arm the restart
  // loop without touching mutex_: a restart attempt that fails swallows its
  // own bus error (schedule() refuses callbacks while callback_running_), and
  // a dead pipeline emits no further messages — the 10 ms push cadence is what
  // keeps the ~4 restarts/sec loop alive.
  std::atomic<bool> pipeline_active_{false};

  // 0 = unclaimed; otherwise the owning reader_id. Claim/release only via CAS.
  std::atomic<std::uint64_t> owner_{0};
  std::atomic<bool> is_shutdown_{false};

  // Last reader id that produced a dropped-frame log, so a chatty non-owner is
  // logged once rather than per frame. The max() sentinel can never be a real
  // reader id (ids start at 1), so the first genuine drop always logs. Bounded:
  // one word, not one set entry per reader for the process lifetime.
  std::atomic<std::uint64_t> last_ignored_reader_{std::numeric_limits<std::uint64_t>::max()};

  utils::PipelineFailureHandler failure_handler_;
};

}  // namespace livekit_ros2_bridge::audio
