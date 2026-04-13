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

#include <cstdint>
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

class VideoFrameSink
{
public:
  virtual ~VideoFrameSink() = default;

  virtual void write(
    int frame_width, int frame_height, std::vector<std::uint8_t> i420_frame, std::int64_t frame_timestamp_us) = 0;
};

// VideoStreamInstance owns one frame source on the input side and wires it to a
// VideoTrackPublisher through this sink interface. Sources produce frames into the sink.
class VideoFrameSource
{
public:
  virtual ~VideoFrameSource() = default;

  virtual void start() = 0;
  virtual void shutdown() = 0;
};

class VideoStreamLifecycleObserver
{
public:
  virtual ~VideoStreamLifecycleObserver() = default;

  virtual void onTrackPublished(int width, int height, bool republished) = 0;
  // Called immediately before the current published track is unpublished.
  virtual void onTrackUnpublishing() = 0;
  virtual void onSampleUnpackFailed(const std::string & error) = 0;
  virtual void onCaptureFailed(const std::string & error) = 0;
  virtual void onPipelineFailed(const std::string & reason) = 0;
  virtual void onRestartFailed(const std::string & error) = 0;
  virtual void onPushFailed(const std::string & error) = 0;
};

}  // namespace livekit_ros2_bridge
