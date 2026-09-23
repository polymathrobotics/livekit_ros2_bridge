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

#include "audio/stream_spec.hpp"

#include <stdexcept>
#include <string>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/percent_encode.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::audio
{

namespace
{

constexpr char kOtherAudioKeyPrefix[] = "other_audio";
constexpr char kOtherTrackPrefix[] = "lkros.audio.other.";
const auto kLogger = rclcpp::get_logger("audio_stream_spec");

}  // namespace

StreamSpec resolveOtherSourceSpec(const StreamConfig & config, const std::string & source_name)
{
  const std::string name = trim(source_name);
  if (name.empty()) {
    throw std::invalid_argument("Invalid other audio name.");
  }

  const auto it = config.other_sources.find(name);
  if (it == config.other_sources.end()) {
    throw std::invalid_argument("Unknown other audio source '" + name + "'.");
  }

  const auto & source = it->second;

  StreamSpec spec;
  spec.stream_key = std::string{kOtherAudioKeyPrefix} + ":" + name;
  spec.track_name = std::string{kOtherTrackPrefix} + utils::percentEncodeUnreserved(name);
  spec.input = OtherInput{name, source.source_fragment, source.transform_fragment};
  spec.publish_options = source.publish_options;
  return spec;
}

}  // namespace livekit_ros2_bridge::audio
