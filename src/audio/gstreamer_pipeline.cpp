// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "audio/gstreamer_pipeline.hpp"

#include <gst/audio/audio.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "audio/pipeline_description.hpp"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

livekit::AudioFrame unpackAudioFrame(GstSample * sample)
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (caps == nullptr || buffer == nullptr) {
    throw std::runtime_error("GStreamer sample is missing caps or buffer.");
  }

  GstAudioInfo info;
  if (!gst_audio_info_from_caps(&info, caps)) {
    throw std::runtime_error("Failed to parse GStreamer audio caps.");
  }
  if (GST_AUDIO_INFO_FORMAT(&info) != GST_AUDIO_FORMAT_S16) {
    throw std::runtime_error("Audio pipeline did not output S16 frames.");
  }

  const int rate = static_cast<int>(GST_AUDIO_INFO_RATE(&info));
  const int channels = static_cast<int>(GST_AUDIO_INFO_CHANNELS(&info));
  if (rate <= 0 || channels <= 0) {
    throw std::runtime_error("Audio sample rate or channel count is invalid.");
  }

  utils::GstBufferMap mapping(*buffer, GST_MAP_READ);
  if (!mapping.is_valid()) {
    throw std::runtime_error("Failed to map GStreamer audio buffer.");
  }

  const auto * info_map = mapping.get();
  const auto * data = static_cast<const std::int16_t *>(static_cast<const void *>(info_map->data));
  const std::size_t sample_count = info_map->size / sizeof(std::int16_t);
  if (sample_count == 0U || sample_count % static_cast<std::size_t>(channels) != 0U) {
    throw std::runtime_error("Audio sample count is inconsistent with the channel count.");
  }

  std::vector<std::int16_t> samples(data, data + sample_count);
  return livekit::AudioFrame(
    std::move(samples), rate, channels, static_cast<int>(sample_count / static_cast<std::size_t>(channels)));
}

}  // namespace

GStreamerPipeline::GStreamerPipeline(PipelineCallbacks callbacks)
{
  callbacks_ = std::make_unique<PipelineCallbacks>(std::move(callbacks));
  callbacks_ptr_.store(callbacks_.get(), std::memory_order_release);
}

GStreamerPipeline::~GStreamerPipeline()
{
  stop();
}

bool GStreamerPipeline::isActive() const noexcept
{
  return pipeline_ != nullptr;
}

PipelineCallbacks GStreamerPipeline::callbacks() const
{
  const auto * callbacks = callbacks_ptr_.load(std::memory_order_acquire);
  return callbacks == nullptr ? PipelineCallbacks{} : *callbacks;
}

// GStreamer may enter raw C callbacks from streaming/bus threads while stop()
// clears callback pointers, so this tight atomic gate intentionally stays local.
bool GStreamerPipeline::beginCallback()
{
  std::size_t state = callback_state_.load(std::memory_order_acquire);
  while ((state & kCallbacksStopped) == 0U) {
    if (callback_state_.compare_exchange_weak(state, state + 1U, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      return true;
    }
  }
  return false;
}

void GStreamerPipeline::endCallback()
{
  (void)callback_state_.fetch_sub(1U, std::memory_order_acq_rel);
}

void GStreamerPipeline::resumeCallbacks()
{
  callback_state_.store(0U, std::memory_order_release);
}

void GStreamerPipeline::stopCallbacksAndWait()
{
  std::size_t state = callback_state_.fetch_or(kCallbacksStopped, std::memory_order_acq_rel) | kCallbacksStopped;
  while ((state & kCallbackCountMask) != 0U) {
    std::this_thread::yield();
    state = callback_state_.load(std::memory_order_acquire);
  }
}

void GStreamerPipeline::start(const std::string & description)
{
  utils::ensureGStreamerInitialized();
  stop();
  resumeCallbacks();

  GError * raw_error = nullptr;
  utils::GstElementPtr pipeline(gst_parse_launch(description.c_str(), &raw_error));
  utils::GErrorPtr error(raw_error);
  if (pipeline == nullptr) {
    const std::string message = error != nullptr ? error->message : "gst_parse_launch returned null";
    throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
  }
  if (!GST_IS_BIN(pipeline.get())) {
    throw std::runtime_error("Audio pipeline must resolve to a GstBin.");
  }

  utils::GstElementPtr sink_element(gst_bin_get_by_name(GST_BIN(pipeline.get()), kBridgeAppSinkName));
  if (sink_element == nullptr) {
    throw std::runtime_error("Audio pipeline did not create the expected appsink.");
  }
  if (!GST_IS_APP_SINK(sink_element.get())) {
    throw std::runtime_error(std::string("Audio pipeline named ") + kBridgeAppSinkName + " must be a GstAppSink.");
  }

  GstAppSinkPtr appsink(GST_APP_SINK(sink_element.release()));
  // stop() clears raw pipeline callbacks before releasing the bin.
  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = [](GstAppSink * sink, gpointer user_data) -> GstFlowReturn {
    return static_cast<GStreamerPipeline *>(user_data)->onSample(sink);
  };
  gst_app_sink_set_callbacks(appsink.get(), &callbacks, this, nullptr);

  utils::GstBusPtr bus(gst_element_get_bus(pipeline.get()));
  gst_bus_set_sync_handler(
    bus.get(),
    [](GstBus *, GstMessage * message, gpointer user_data) -> GstBusSyncReply {
      static_cast<GStreamerPipeline *>(user_data)->onBusMessage(message);
      return GST_BUS_PASS;
    },
    this,
    nullptr);

  pipeline_ = std::move(pipeline);

  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (result == GST_STATE_CHANGE_FAILURE) {
    stop();
    throw std::runtime_error("Failed to set audio pipeline to PLAYING.");
  }
}

void GStreamerPipeline::stop()
{
  if (pipeline_ == nullptr) {
    return;
  }

  utils::GstElementPtr appsink(gst_bin_get_by_name(GST_BIN(pipeline_.get()), kBridgeAppSinkName));
  if (appsink != nullptr && GST_IS_APP_SINK(appsink.get())) {
    GstAppSinkCallbacks callbacks{};
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink.get()), &callbacks, nullptr, nullptr);
  }

  utils::GstBusPtr bus(gst_element_get_bus(pipeline_.get()));
  gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);
  const GstStateChangeReturn result = gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  if (result == GST_STATE_CHANGE_ASYNC) {
    (void)gst_element_get_state(pipeline_.get(), nullptr, nullptr, GST_CLOCK_TIME_NONE);
  }
  stopCallbacksAndWait();

  pipeline_.reset();
}

GstFlowReturn GStreamerPipeline::onSample(GstAppSink * sink)
{
  if (!beginCallback()) {
    return GST_FLOW_FLUSHING;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  const auto callbacks = this->callbacks();
  utils::GstSamplePtr sample(gst_app_sink_pull_sample(sink));
  if (sample == nullptr) {
    return GST_FLOW_EOS;
  }

  livekit::AudioFrame frame;
  try {
    frame = unpackAudioFrame(sample.get());
  } catch (const std::exception & exc) {
    callbacks.on_unpack_failed(exc.what());
    return GST_FLOW_ERROR;
  }

  if (callbacks.is_shutdown()) {
    return GST_FLOW_FLUSHING;
  }

  try {
    callbacks.on_frame(frame);
    return GST_FLOW_OK;
  } catch (const std::exception & exc) {
    callbacks.on_capture_failed(exc.what());
    return GST_FLOW_ERROR;
  }
}

void GStreamerPipeline::onBusMessage(GstMessage * message)
{
  if (!beginCallback()) {
    return;
  }
  const ScopeExit callback_exit([this]() { endCallback(); });
  const auto callbacks = this->callbacks();
  const GstMessageType type = GST_MESSAGE_TYPE(message);
  if (type == GST_MESSAGE_EOS) {
    callbacks.on_failed("eos");
    return;
  }
  if (type != GST_MESSAGE_ERROR) {
    return;
  }

  GError * raw_error = nullptr;
  gst_message_parse_error(message, &raw_error, nullptr);
  utils::GErrorPtr error(raw_error);
  const std::string reason = error != nullptr && error->message != nullptr ? error->message : "error";
  callbacks.on_failed(reason);
}

}  // namespace livekit_ros2_bridge::audio
