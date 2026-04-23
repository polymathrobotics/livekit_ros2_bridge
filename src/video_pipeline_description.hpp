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

#include <string>

namespace livekit_ros2_bridge
{

inline constexpr char kBridgeAppSrcName[] = "bridge_video_src";
inline constexpr char kBridgeAppSinkName[] = "bridge_video_sink";

inline std::string buildPipelineDescription(const std::string & ingress, const std::string & transform)
{
  std::string description = ingress;
  if (!transform.empty()) {
    description += " ! ";
    description += transform;
  }
  // Keep only a tiny downstream queue so backpressure drops stale frames instead
  // of letting latency grow without bound.
  description += " ! queue max-size-buffers=2 leaky=downstream";
  description += " ! videoconvert";
  // Emit encoder-native I420 so LiveKit/WebRTC does not have to do an extra
  // RGBA->I420 conversion on every captured frame.
  description += " ! video/x-raw,format=I420";
  description += " ! appsink name=";
  description += kBridgeAppSinkName;
  description += " sync=false drop=true max-buffers=1 emit-signals=false";
  return description;
}

}  // namespace livekit_ros2_bridge
