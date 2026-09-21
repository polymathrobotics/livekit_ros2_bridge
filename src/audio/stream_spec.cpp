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

#include "audio/stream_spec.hpp"

#include <stdexcept>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/percent_encode.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

constexpr char kExternalAudioKeyPrefix[] = "external_audio";
constexpr char kExternalTrackPrefix[] = "lkros.audio.external.";
const auto kLogger = rclcpp::get_logger("audio_stream_spec");

}  // namespace

StreamSpec resolveExternalSourceSpec(const StreamConfig & config, const std::string & source_name)
{
  const std::string name = trim(source_name);
  if (name.empty()) {
    throw std::invalid_argument("Invalid external audio name.");
  }

  const auto it = config.external_sources.find(name);
  if (it == config.external_sources.end()) {
    throw std::invalid_argument("Unknown external audio source '" + name + "'.");
  }

  const auto & source = it->second;

  StreamSpec spec;
  spec.stream_key = std::string{kExternalAudioKeyPrefix} + ":" + name;
  spec.track_name = std::string{kExternalTrackPrefix} + utils::percentEncodeUnreserved(name);
  spec.input = ExternalInput{name, source.source_fragment, source.transform_fragment};
  spec.publish_options = source.publish_options;
  return spec;
}

}  // namespace livekit_ros2_bridge::audio
