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

#include <memory>
#include <mutex>

namespace livekit_ros2_bridge::utils
{

struct GstObjectDeleter
{
  template <typename T>
  void operator()(T * object) const
  {
    gst_object_unref(object);
  }
};

struct GErrorDeleter
{
  void operator()(GError * object) const
  {
    g_error_free(object);
  }
};

struct GCharDeleter
{
  void operator()(gchar * object) const
  {
    g_free(object);
  }
};

struct GstIteratorDeleter
{
  void operator()(GstIterator * object) const
  {
    gst_iterator_free(object);
  }
};

struct GstCapsDeleter
{
  void operator()(GstCaps * object) const
  {
    // GstCaps is a GstMiniObject, not a GObject: it must be released with
    // gst_caps_unref, never gst_object_unref/g_object_unref.
    gst_caps_unref(object);
  }
};

struct GstBufferDeleter
{
  void operator()(GstBuffer * object) const
  {
    gst_buffer_unref(object);
  }
};

struct GstSampleDeleter
{
  void operator()(GstSample * object) const
  {
    gst_sample_unref(object);
  }
};

template <typename T>
using GstObjectPtr = std::unique_ptr<T, GstObjectDeleter>;

using GstElementPtr = GstObjectPtr<GstElement>;
using GstBusPtr = GstObjectPtr<GstBus>;
using GstCapsPtr = std::unique_ptr<GstCaps, GstCapsDeleter>;
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

}  // namespace livekit_ros2_bridge::utils
