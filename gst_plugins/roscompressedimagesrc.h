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

#include <gst/base/gstpushsrc.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "ros_node_runner.hpp"

G_BEGIN_DECLS

#define GST_TYPE_ROSCOMPRESSEDIMAGESRC (roscompressedimagesrc_get_type())
#define GST_ROSCOMPRESSEDIMAGESRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSCOMPRESSEDIMAGESRC, RosCompressedImageSrc))
#define GST_ROSCOMPRESSEDIMAGESRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSCOMPRESSEDIMAGESRC, RosCompressedImageSrcClass))

typedef struct _RosCompressedImageSrc RosCompressedImageSrc;
typedef struct _RosCompressedImageSrcClass RosCompressedImageSrcClass;

struct _RosCompressedImageSrc
{
  GstPushSrc parent;

  gchar * ros_topic;
  gboolean ros_reliable;

  // GObject allocates the instance as raw storage, so these C++ members are
  // placement-new'd in init() and destroyed manually in finalize().
  std::unique_ptr<RosNodeRunner> node_runner;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;

  // The subscription callback keeps only the newest frame. Both getcaps() and
  // create() wait on this queue, then decide when to drop stale entries.
  std::deque<sensor_msgs::msg::CompressedImage::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;

  // flushing wakes blocked waiters during unlock() and makes them return
  // GST_FLOW_FLUSHING instead of consuming queued data.
  gboolean flushing;
  // Caps are latched from the first negotiated frame and held until the element
  // returns to READY/NULL.
  gboolean caps_set;
  std::string detected_format;
};

struct _RosCompressedImageSrcClass
{
  GstPushSrcClass parent_class;
};

GType roscompressedimagesrc_get_type(void);

G_END_DECLS
