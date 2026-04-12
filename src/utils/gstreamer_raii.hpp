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

namespace livekit_ros2_bridge
{

// GStreamer and GLib expose mostly C-style ownership: callers receive raw pointers,
// ref-counted objects, or mapped buffers and must remember the matching cleanup call.
// These wrappers bind those cleanup rules to C++ scope lifetime so call sites can
// fail or return early without duplicating teardown code.

// Most long-lived GStreamer runtime objects inherit from GstObject and are released
// by dropping a reference with gst_object_unref().
struct GstObjectDeleter
{
  template <typename T>
  void operator()(T * ptr) const
  {
    if (ptr != nullptr) {
      gst_object_unref(ptr);
    }
  }
};

// gst_parse_launch() and several message-parsing APIs allocate GError instances
// that must be released with g_error_free().
struct GErrorDeleter
{
  void operator()(GError * ptr) const
  {
    if (ptr != nullptr) {
      g_error_free(ptr);
    }
  }
};

// GLib allocates many returned strings with g_malloc(); g_free() is the matching
// release function for those gchar* results.
struct GCharDeleter
{
  void operator()(gchar * ptr) const
  {
    if (ptr != nullptr) {
      g_free(ptr);
    }
  }
};

// Bin traversal returns an iterator handle that must be freed explicitly.
struct GstIteratorDeleter
{
  void operator()(GstIterator * ptr) const
  {
    if (ptr != nullptr) {
      gst_iterator_free(ptr);
    }
  }
};

// Buffers and samples are ref-counted, but they do not inherit from GstObject.
struct GstBufferDeleter
{
  void operator()(GstBuffer * ptr) const
  {
    if (ptr != nullptr) {
      gst_buffer_unref(ptr);
    }
  }
};

struct GstSampleDeleter
{
  void operator()(GstSample * ptr) const
  {
    if (ptr != nullptr) {
      gst_sample_unref(ptr);
    }
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

inline void ensureGstreamerInitialized()
{
  static std::once_flag init_once;
  std::call_once(init_once, []() { gst_init(nullptr, nullptr); });
}

// GstIterator writes each result into a GValue. The value has to be reset between
// iterations and fully unset before leaving scope to release any embedded refs.
class GValueGuard final
{
public:
  GValueGuard() = default;

  ~GValueGuard()
  {
    unset();
  }

  GValueGuard(const GValueGuard &) = delete;
  GValueGuard & operator=(const GValueGuard &) = delete;
  GValueGuard(GValueGuard &&) = delete;
  GValueGuard & operator=(GValueGuard &&) = delete;

  GValue * get()
  {
    return &value_;
  }

  const GValue * get() const
  {
    return &value_;
  }

  void reset()
  {
    // reset() keeps the GValue allocated but releases the contained payload,
    // which matches GStreamer's iterator usage pattern.
    if (G_IS_VALUE(&value_)) {
      g_value_reset(&value_);
    }
  }

  void unset()
  {
    if (G_IS_VALUE(&value_)) {
      g_value_unset(&value_);
      value_ = GValue{};
    }
  }

private:
  GValue value_ = G_VALUE_INIT;
};

// gst_buffer_map() pins a buffer's memory until gst_buffer_unmap() is called.
// This guard makes that pairing exception-safe for both read and write paths.
class GstMapGuard final
{
public:
  GstMapGuard(GstBuffer * buffer, GstMapFlags flags)
  : buffer_(buffer)
  , mapped_(buffer_ != nullptr && gst_buffer_map(buffer_, &info_, flags))
  {}

  ~GstMapGuard()
  {
    if (mapped_) {
      gst_buffer_unmap(buffer_, &info_);
    }
  }

  GstMapGuard(const GstMapGuard &) = delete;
  GstMapGuard & operator=(const GstMapGuard &) = delete;
  GstMapGuard(GstMapGuard &&) = delete;
  GstMapGuard & operator=(GstMapGuard &&) = delete;

  bool is_valid() const
  {
    return mapped_;
  }

  GstMapInfo * get()
  {
    return &info_;
  }

  const GstMapInfo * get() const
  {
    return &info_;
  }

private:
  GstBuffer * buffer_ = nullptr;
  GstMapInfo info_{};
  bool mapped_ = false;
};

// gst_video_frame_map() exposes per-plane image data with stride metadata until
// gst_video_frame_unmap() is called. This guard keeps planar frame access scoped
// and exception-safe.
class GstVideoFrameGuard final
{
public:
  GstVideoFrameGuard(const GstVideoInfo * info, GstBuffer * buffer, GstMapFlags flags)
  : mapped_(info != nullptr && buffer != nullptr && gst_video_frame_map(&frame_, info, buffer, flags))
  {}

  ~GstVideoFrameGuard()
  {
    if (mapped_) {
      gst_video_frame_unmap(&frame_);
    }
  }

  GstVideoFrameGuard(const GstVideoFrameGuard &) = delete;
  GstVideoFrameGuard & operator=(const GstVideoFrameGuard &) = delete;
  GstVideoFrameGuard(GstVideoFrameGuard &&) = delete;
  GstVideoFrameGuard & operator=(GstVideoFrameGuard &&) = delete;

  bool is_valid() const
  {
    return mapped_;
  }

  GstVideoFrame * get()
  {
    return &frame_;
  }

  const GstVideoFrame * get() const
  {
    return &frame_;
  }

private:
  GstVideoFrame frame_{};
  bool mapped_ = false;
};

}  // namespace livekit_ros2_bridge
