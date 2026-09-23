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

#pragma once

#include <chrono>
#include <string>
#include <string_view>
#include <unordered_map>

#include "livekit/room_event_types.h"

namespace livekit_ros2_bridge::audio
{

struct OtherSource
{
  std::string source_fragment;
  std::string transform_fragment;
  livekit::TrackPublishOptions publish_options;
};

struct StreamConfig
{
  // Keyed by the trimmed configured source name.
  std::unordered_map<std::string, OtherSource> other_sources;
  livekit::TrackPublishOptions default_publish_options;
};

inline StreamConfig makeDefaultConfig()
{
  return StreamConfig{};
}

struct OtherInput
{
  std::string name;
  std::string source_fragment;
  std::string transform_fragment;
};

struct StreamSpec
{
  // Stable runtime key: "other_audio:<trimmed source name>".
  std::string stream_key;
  // LiveKit track name: reversible percent-encoded other-audio suffix.
  std::string track_name;

  OtherInput input;
  livekit::TrackPublishOptions publish_options;
};

StreamSpec resolveOtherSourceSpec(const StreamConfig & config, const std::string & source_name);

}  // namespace livekit_ros2_bridge::audio
