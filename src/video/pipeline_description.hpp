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

#include "livekit/video_codec.h"

namespace livekit_ros2_bridge::video
{

// Reserved names used to recover bridge-owned endpoints from parsed GStreamer bins.
inline constexpr char kBridgeAppSrcName[] = "bridge_video_src";
inline constexpr char kBridgeAppSinkName[] = "bridge_video_sink";

inline constexpr auto kBridgeQueueDuration = std::chrono::milliseconds(500);

// The layout is what the LiveKit SDK expects: one access unit (AV1: temporal unit) per buffer.
struct PassthroughFormat
{
  livekit::VideoCodec codec;
  // Short codec name used in log fields.
  const char * name;
  const char * media_type;
  const char * layout;
};

inline constexpr PassthroughFormat kPassthroughFormats[] = {
  {livekit::VideoCodec::H264, "h264", "video/x-h264", "stream-format=byte-stream,alignment=au"},
  {livekit::VideoCodec::H265, "h265", "video/x-h265", "stream-format=byte-stream,alignment=au"},
  {livekit::VideoCodec::AV1, "av1", "video/x-av1", "stream-format=obu-stream,alignment=tu"},
};

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
  // Sources must end with their codec's parser, or this fails to link or negotiate.
  std::string caps;
  for (const PassthroughFormat & format : kPassthroughFormats) {
    caps += caps.empty() ? "" : ";";
    caps += format.media_type;
    caps += ",";
    caps += format.layout;
  }
  description += " ! ";
  description += caps;
  // Drop the oldest frames rather than block the source (the next frame is marked DISCONT). The
  // 30-buffer cap bounds sources without timestamps; the 16 MiB cap only bounds memory.
  description += " ! queue leaky=downstream max-size-time=";
  description += std::to_string(std::chrono::nanoseconds(kBridgeQueueDuration).count());
  description += " max-size-buffers=30 max-size-bytes=16777216";
  description += " ! appsink name=";
  description += kBridgeAppSinkName;
  description += " sync=false";
  return description;
}

}  // namespace livekit_ros2_bridge::video
