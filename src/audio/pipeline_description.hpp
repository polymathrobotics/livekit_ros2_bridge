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

#include <string>

namespace livekit_ros2_bridge::audio
{

// Reserved name used to recover the bridge-owned appsink from parsed GStreamer bins.
inline constexpr char kBridgeAppSinkName[] = "bridge_audio_sink";

// The bridge owns the pipeline edge: it appends a fixed tail that forces mono
// 48 kHz S16 (WebRTC/Opus's native rate) regardless of the source, and is leaky
// so the newest audio wins under backpressure.
inline std::string buildPipelineDescription(const std::string & source, const std::string & transform)
{
  std::string description = source;
  if (!transform.empty()) {
    description += " ! ";
    description += transform;
  }
  description += " ! queue max-size-time=100000000 leaky=downstream";
  description += " ! audioconvert";
  description += " ! audioresample";
  description += " ! audio/x-raw,format=S16LE,channels=1,rate=48000";
  description += " ! appsink name=";
  description += kBridgeAppSinkName;
  description += " sync=false drop=true max-buffers=1";
  return description;
}

}  // namespace livekit_ros2_bridge::audio
