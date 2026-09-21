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

#include "audio/stream_config.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "audio/pipeline_description.hpp"
#include "livekit/room_event_types.h"
#include "utils/gstreamer_pipeline_validation.hpp"
#include "utils/param_entries.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

void setEncoding(livekit::TrackPublishOptions & options, std::uint64_t max_bitrate_bps)
{
  if (max_bitrate_bps > 0) {
    livekit::AudioEncodingOptions encoding;
    encoding.max_bitrate = max_bitrate_bps;
    options.audio_encoding = encoding;
    return;
  }

  options.audio_encoding.reset();
}

livekit::TrackPublishOptions parsePublishOptions(const Params & params)
{
  livekit::TrackPublishOptions options;
  setEncoding(options, static_cast<std::uint64_t>(params.audio.publish.max_bitrate_bps));
  const std::string dtx = trim(params.audio.publish.dtx);
  if (dtx == "enabled") {
    options.dtx = true;
  } else if (dtx == "disabled") {
    options.dtx = false;
  }
  const std::string red = trim(params.audio.publish.red);
  if (red == "enabled") {
    options.red = true;
  } else if (red == "disabled") {
    options.red = false;
  }
  return options;
}

template <typename EntryT>
livekit::TrackPublishOptions parsePublishOptions(const EntryT & entry, const livekit::TrackPublishOptions & defaults)
{
  livekit::TrackPublishOptions options = defaults;

  // Negative values mean "inherit global default"; the generated parameter
  // schema cannot express optional scalars for these fields.
  std::uint64_t max_bitrate_bps = 0;
  if (options.audio_encoding.has_value()) {
    max_bitrate_bps = options.audio_encoding->max_bitrate;
  }
  if (entry.publish.max_bitrate_bps >= 0) {
    max_bitrate_bps = static_cast<std::uint64_t>(entry.publish.max_bitrate_bps);
  }
  setEncoding(options, max_bitrate_bps);

  const std::string dtx = trim(entry.publish.dtx);
  if (dtx == "enabled") {
    options.dtx = true;
  } else if (dtx == "disabled") {
    options.dtx = false;
  }
  const std::string red = trim(entry.publish.red);
  if (red == "enabled") {
    options.red = true;
  } else if (red == "disabled") {
    options.red = false;
  }

  return options;
}

using utils::requireUniqueEntry;

}  // namespace

StreamConfig loadConfig(const Params & params)
{
  StreamConfig config = makeDefaultConfig();
  config.default_publish_options = parsePublishOptions(params);

  std::unordered_set<std::string> seen_source_ids;
  std::unordered_set<std::string> seen_source_names;
  constexpr utils::EndpointLayout kExternalSourceLayout = utils::makeExternalSourceLayout(kBridgeAppSinkName);

  for (const auto & id : params.audio_external_ids) {
    const auto & entry = requireUniqueEntry(
      seen_source_ids, id, params.audio.external.audio_external_ids_map, "external audio id", "external audio source");

    const std::string source_context = "external audio source '" + id + "'";
    const std::string source_fragment = trim(entry.source);
    if (source_fragment.empty()) {
      throw std::runtime_error(source_context + " requires a non-empty source");
    }
    const std::string transform = trim(entry.transform);
    utils::validatePipeline(
      source_context, buildPipelineDescription(source_fragment, transform), kExternalSourceLayout);

    // Only trim surrounding whitespace; slash and colon variants stay distinct.
    const std::string name = trim(id);
    if (name.empty()) {
      throw std::runtime_error(source_context + " must trim to a non-empty name");
    }
    if (!seen_source_names.emplace(name).second) {
      throw std::runtime_error("duplicate external audio source name '" + name + "'");
    }

    ExternalSource source;
    source.source_fragment = source_fragment;
    source.transform_fragment = transform;
    source.publish_options = parsePublishOptions(entry, config.default_publish_options);
    config.external_sources.emplace(name, std::move(source));
  }

  return config;
}

}  // namespace livekit_ros2_bridge::audio
