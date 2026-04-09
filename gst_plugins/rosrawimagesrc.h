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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_node_runner.hpp"

#include <gst/video/video-format.h>

G_BEGIN_DECLS

#define GST_TYPE_ROSRAWIMAGESRC (rosrawimagesrc_get_type())
#define GST_ROSRAWIMAGESRC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSRAWIMAGESRC, RosRawImageSrc))
#define GST_ROSRAWIMAGESRC_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSRAWIMAGESRC, RosRawImageSrcClass))

typedef struct _RosRawImageSrc RosRawImageSrc;
typedef struct _RosRawImageSrcClass RosRawImageSrcClass;

struct _RosRawImageSrc
{
  GstPushSrc parent;

  gchar * ros_topic;
  gboolean ros_reliable;

  // GObject allocates the instance as raw storage, so these C++ members are
  // placement-new'd in init() and destroyed manually in finalize().
  std::unique_ptr<RosNodeRunner> node_runner;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

  // The subscription callback keeps only the newest frame. Both getcaps() and
  // create() wait on this queue, then decide when to drop stale entries.
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;

  // flushing wakes blocked waiters during unlock() and makes them return
  // GST_FLOW_FLUSHING instead of consuming queued data.
  gboolean flushing;
  // Caps are latched from the first negotiated frame and held until the element
  // returns to READY/NULL.
  gboolean caps_set;
  int width;
  int height;
  GstVideoFormat format;
  guint64 dropped_frames_since_log;
  gint64 next_drop_log_time_us;
};

struct _RosRawImageSrcClass
{
  GstPushSrcClass parent_class;
};

GType rosrawimagesrc_get_type(void);

G_END_DECLS
