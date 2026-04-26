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

#include <gst/gst.h>
#include <gst/video/video.h>

#include <memory>
#include <mutex>

namespace livekit_ros2_bridge::video
{

struct GstObjectDeleter
{
  template <typename T>
  void operator()(T * ptr) const
  {
    gst_object_unref(ptr);
  }
};

struct GErrorDeleter
{
  void operator()(GError * ptr) const
  {
    g_error_free(ptr);
  }
};

struct GCharDeleter
{
  void operator()(gchar * ptr) const
  {
    g_free(ptr);
  }
};

struct GstIteratorDeleter
{
  void operator()(GstIterator * ptr) const
  {
    gst_iterator_free(ptr);
  }
};

struct GstBufferDeleter
{
  void operator()(GstBuffer * ptr) const
  {
    gst_buffer_unref(ptr);
  }
};

struct GstSampleDeleter
{
  void operator()(GstSample * ptr) const
  {
    gst_sample_unref(ptr);
  }
};

template <typename T>
using GstObjectPtr = std::unique_ptr<T, GstObjectDeleter>;

using GstElementPtr = GstObjectPtr<GstElement>;
using GstBusPtr = GstObjectPtr<GstBus>;
using GErrorPtr = std::unique_ptr<GError, GErrorDeleter>;
using GCharPtr = std::unique_ptr<gchar, GCharDeleter>;
using GstIteratorPtr = std::unique_ptr<GstIterator, GstIteratorDeleter>;
using GstBufferPtr = std::unique_ptr<GstBuffer, GstBufferDeleter>;
using GstSamplePtr = std::unique_ptr<GstSample, GstSampleDeleter>;

// Thread-safe process-wide initialization for call sites that may touch
// GStreamer before the node runtime has established ordering.
inline void ensureGStreamerInitialized()
{
  static std::once_flag init_once;
  std::call_once(init_once, []() { gst_init(nullptr, nullptr); });
}

// Owns GstIterator's reusable GValue slot. Call reset() after consuming
// GST_ITERATOR_OK; destruction unsets any remaining payload.
class GValueSlot final
{
public:
  GValueSlot() = default;

  ~GValueSlot()
  {
    if (G_IS_VALUE(&value_)) {
      g_value_unset(&value_);
      value_ = GValue{};
    }
  }

  GValueSlot(const GValueSlot &) = delete;
  GValueSlot & operator=(const GValueSlot &) = delete;
  GValueSlot(GValueSlot &&) = delete;
  GValueSlot & operator=(GValueSlot &&) = delete;

  GValue * get()
  {
    return &value_;
  }

  void reset()
  {
    if (G_IS_VALUE(&value_)) {
      g_value_reset(&value_);
    }
  }

private:
  GValue value_ = G_VALUE_INIT;
};

class GstBufferMap final
{
public:
  GstBufferMap(GstBuffer & buffer, GstMapFlags flags)
  : buffer_(&buffer)
  , mapped_(gst_buffer_map(buffer_, &info_, flags))
  {}

  ~GstBufferMap()
  {
    if (mapped_) {
      gst_buffer_unmap(buffer_, &info_);
    }
  }

  GstBufferMap(const GstBufferMap &) = delete;
  GstBufferMap & operator=(const GstBufferMap &) = delete;
  GstBufferMap(GstBufferMap &&) = delete;
  GstBufferMap & operator=(GstBufferMap &&) = delete;

  bool is_valid() const
  {
    return mapped_;
  }

  GstMapInfo * get()
  {
    return &info_;
  }

private:
  GstBuffer * buffer_ = nullptr;
  GstMapInfo info_{};
  bool mapped_ = false;
};

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
