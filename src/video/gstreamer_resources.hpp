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

#include <gst/video/video.h>

#include "utils/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::video
{

using utils::ensureGStreamerInitialized;
using utils::GCharDeleter;
using utils::GCharPtr;
using utils::GErrorDeleter;
using utils::GErrorPtr;
using utils::GstBufferDeleter;
using utils::GstBufferMap;
using utils::GstBufferPtr;
using utils::GstBusPtr;
using utils::GstElementPtr;
using utils::GstIteratorDeleter;
using utils::GstIteratorPtr;
using utils::GstObjectDeleter;
using utils::GstObjectPtr;
using utils::GstSampleDeleter;
using utils::GstSamplePtr;
using utils::GValueSlot;

class GstVideoFrameMap final
{
public:
  GstVideoFrameMap(const GstVideoInfo & info, GstBuffer & buffer, GstMapFlags flags)
  : mapped_(gst_video_frame_map(&frame_, &info, &buffer, flags))
  {}

  ~GstVideoFrameMap()
  {
    if (mapped_) {
      gst_video_frame_unmap(&frame_);
    }
  }

  GstVideoFrameMap(const GstVideoFrameMap &) = delete;
  GstVideoFrameMap & operator=(const GstVideoFrameMap &) = delete;
  GstVideoFrameMap(GstVideoFrameMap &&) = delete;
  GstVideoFrameMap & operator=(GstVideoFrameMap &&) = delete;

  bool is_valid() const
  {
    return mapped_;
  }

  GstVideoFrame * get()
  {
    return &frame_;
  }

private:
  GstVideoFrame frame_{};
  bool mapped_ = false;
};

}  // namespace livekit_ros2_bridge::video
