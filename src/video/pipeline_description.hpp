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

namespace livekit_ros2_bridge::video
{

// Reserved names used to recover bridge-owned endpoints from parsed GStreamer bins.
inline constexpr char kBridgeAppSrcName[] = "bridge_video_src";
inline constexpr char kBridgeAppSinkName[] = "bridge_video_sink";
inline constexpr char kBridgeVideoQueueName[] = "bridge_video_queue";

inline std::string buildPipelineDescription(const std::string & source, const std::string & transform)
{
  std::string description = source;
  if (!transform.empty()) {
    description += " ! ";
    description += transform;
  }
  // Drop stale frames under backpressure instead of adding latency.
  description += " ! queue max-size-buffers=2 leaky=downstream";
  description += " ! videoconvert";
  // Keep LiveKit/WebRTC on I420 to avoid per-frame RGBA conversion.
  description += " ! video/x-raw,format=I420";
  description += " ! appsink name=";
  description += kBridgeAppSinkName;
  description += " sync=false drop=true max-buffers=1";
  return description;
}

inline std::string buildPassthroughPipelineDescription(const std::string & source)
{
  std::string description = source;
  // Repeat SPS/PPS on every IDR so a new viewer can start from any keyframe, and
  // emit the Annex-B, one-picture-per-buffer form the LiveKit SDK expects.
  description += " ! h264parse config-interval=-1";
  description += " ! video/x-h264,stream-format=byte-stream,alignment=au";
  // Never drop: a lost encoded frame corrupts the picture until the next keyframe. An overrun of
  // this ~1 s queue is the "fell behind" signal; the 32 MiB cap backstops buffers without
  // timestamps, which never reach the time limit.
  description += " ! queue name=";
  description += kBridgeVideoQueueName;
  description += " max-size-buffers=0 max-size-bytes=33554432 max-size-time=1000000000 leaky=no";
  description += " ! appsink name=";
  description += kBridgeAppSinkName;
  description += " sync=false";
  return description;
}

}  // namespace livekit_ros2_bridge::video
