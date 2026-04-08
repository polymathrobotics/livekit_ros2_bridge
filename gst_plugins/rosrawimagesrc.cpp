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

#include "rosrawimagesrc.h"

#include "encoding_utils.hpp"

GST_DEBUG_CATEGORY_STATIC(rosrawimagesrc_debug);
#define GST_CAT_DEFAULT rosrawimagesrc_debug

enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_RELIABLE,
};

#define RAW_IMAGE_CAPS                  \
  "video/x-raw, "                       \
  "format = " GST_ROS_VIDEO_FORMAT_LIST \
  ", "                                  \
  "framerate = " GST_VIDEO_FPS_RANGE    \
  ", "                                  \
  "width = " GST_VIDEO_SIZE_RANGE       \
  ", "                                  \
  "height = " GST_VIDEO_SIZE_RANGE

static GstStaticPadTemplate src_template =
  GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS(RAW_IMAGE_CAPS));

G_DEFINE_TYPE_WITH_CODE(
  RosRawImageSrc,
  rosrawimagesrc,
  GST_TYPE_PUSH_SRC,
  GST_DEBUG_CATEGORY_INIT(rosrawimagesrc_debug, "rosrawimagesrc", 0, "ROS 2 raw image source"))

static void rosrawimagesrc_set_property(GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(object);
  switch (prop_id) {
    case PROP_ROS_TOPIC:
      g_free(self->ros_topic);
      self->ros_topic = g_value_dup_string(value);
      break;
    case PROP_ROS_RELIABLE:
      self->ros_reliable = g_value_get_boolean(value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void rosrawimagesrc_get_property(GObject * object, guint prop_id, GValue * value, GParamSpec * pspec)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(object);
  switch (prop_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, self->ros_topic);
      break;
    case PROP_ROS_RELIABLE:
      g_value_set_boolean(value, self->ros_reliable);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void rosrawimagesrc_finalize(GObject * object)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(object);
  g_free(self->ros_topic);
  self->sub.reset();
  self->node_runner.reset();
  self->msg_queue.~deque();
  self->msg_queue_mtx.~mutex();
  self->msg_queue_cv.~condition_variable();
  G_OBJECT_CLASS(rosrawimagesrc_parent_class)->finalize(object);
}

static sensor_msgs::msg::Image::ConstSharedPtr wait_for_msg(RosRawImageSrc * self)
{
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->msg_queue_cv.wait(lk, [self] { return !self->msg_queue.empty() || self->flushing; });
  if (self->flushing || self->msg_queue.empty()) {
    return nullptr;
  }
  return self->msg_queue.front();
}

static gboolean rosrawimagesrc_unlock(GstBaseSrc * base_src)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(base_src);
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->flushing = TRUE;
  self->msg_queue_cv.notify_all();
  return TRUE;
}

static gboolean rosrawimagesrc_unlock_stop(GstBaseSrc * base_src)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(base_src);
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->flushing = FALSE;
  return TRUE;
}

static GstCaps * rosrawimagesrc_getcaps(GstBaseSrc * base_src, GstCaps * filter)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(base_src);

  if (!self->caps_set) {
    if (!self->node_runner) {
      return gst_pad_get_pad_template_caps(base_src->srcpad);
    }

    GST_DEBUG_OBJECT(self, "waiting for first message to determine caps");
    auto msg = wait_for_msg(self);
    if (!msg) {
      return gst_pad_get_pad_template_caps(base_src->srcpad);
    }

    self->width = static_cast<int>(msg->width);
    self->height = static_cast<int>(msg->height);
    self->format = rosEncodingToGstFormat(msg->encoding);

    {
      std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
      self->msg_queue.clear();
    }

    if (self->format == GST_VIDEO_FORMAT_UNKNOWN) {
      GST_ERROR_OBJECT(self, "Unsupported ROS image encoding: %s", msg->encoding.c_str());
      return gst_pad_get_pad_template_caps(base_src->srcpad);
    }

    self->caps_set = TRUE;
  }

  const gchar * format_str = gst_video_format_to_string(self->format);
  GstCaps * caps = gst_caps_new_simple(
    "video/x-raw",
    "format",
    G_TYPE_STRING,
    format_str,
    "width",
    G_TYPE_INT,
    self->width,
    "height",
    G_TYPE_INT,
    self->height,
    NULL);

  if (filter) {
    GstCaps * intersection = gst_caps_intersect_full(filter, caps, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref(caps);
    return intersection;
  }
  return caps;
}

static GstStateChangeReturn rosrawimagesrc_change_state(GstElement * element, GstStateChange transition)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(element);

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY: {
      try {
        self->node_runner = std::make_unique<RosNodeRunner>("gst_raw_image_src");
      } catch (const std::exception & e) {
        GST_ERROR_OBJECT(self, "Failed to create ROS node: %s", e.what());
        return GST_STATE_CHANGE_FAILURE;
      }

      auto cb = [self](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
        if (self->msg_queue.size() >= 1) {
          self->msg_queue.pop_back();
        }
        self->msg_queue.push_front(msg);
        self->msg_queue_cv.notify_one();
      };

      rclcpp::QoS qos = rclcpp::SensorDataQoS();
      if (self->ros_reliable) {
        qos.reliable();
      }
      self->sub = self->node_runner->node()->create_subscription<sensor_msgs::msg::Image>(self->ros_topic, qos, cb);
      break;
    }

    default:
      break;
  }

  GstStateChangeReturn ret = GST_ELEMENT_CLASS(rosrawimagesrc_parent_class)->change_state(element, transition);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    return ret;
  }

  switch (transition) {
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    case GST_STATE_CHANGE_READY_TO_NULL:
      self->sub.reset();
      self->node_runner.reset();
      self->caps_set = FALSE;
      break;

    default:
      break;
  }

  return ret;
}

static GstFlowReturn rosrawimagesrc_create(GstPushSrc * push_src, GstBuffer ** buf)
{
  RosRawImageSrc * self = GST_ROSRAWIMAGESRC(push_src);

  auto msg = wait_for_msg(self);
  if (!msg) {
    return GST_FLOW_FLUSHING;
  }

  {
    std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
    self->msg_queue.clear();
  }

  if (self->caps_set) {
    int msg_width = static_cast<int>(msg->width);
    int msg_height = static_cast<int>(msg->height);
    GstVideoFormat msg_format = rosEncodingToGstFormat(msg->encoding);

    if (msg_width != self->width || msg_height != self->height || msg_format != self->format) {
      GST_WARNING_OBJECT(
        self,
        "ROS message does not match negotiated caps: expected %dx%d %s, got %dx%d %s",
        self->width,
        self->height,
        gst_video_format_to_string(self->format),
        msg_width,
        msg_height,
        gst_video_format_to_string(msg_format));
      return GST_FLOW_ERROR;
    }
  }

  gsize length = msg->data.size();
  *buf = gst_buffer_new_allocate(NULL, length, NULL);
  if (*buf == NULL) {
    return GST_FLOW_ERROR;
  }

  GstMapInfo info;
  gst_buffer_map(*buf, &info, GST_MAP_WRITE);
  memcpy(info.data, msg->data.data(), length);
  gst_buffer_unmap(*buf, &info);

  return GST_FLOW_OK;
}

static void rosrawimagesrc_init(RosRawImageSrc * self)
{
  self->ros_topic = g_strdup("image");
  self->ros_reliable = FALSE;
  self->flushing = FALSE;
  self->caps_set = FALSE;

  new (&self->msg_queue) std::deque<sensor_msgs::msg::Image::ConstSharedPtr>();
  new (&self->msg_queue_mtx) std::mutex();
  new (&self->msg_queue_cv) std::condition_variable();
  new (&self->node_runner) std::unique_ptr<RosNodeRunner>();
  new (&self->sub) rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr();

  gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp(GST_BASE_SRC(self), TRUE);
}

static void rosrawimagesrc_class_init(RosRawImageSrcClass * klass)
{
  GObjectClass * gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass * basesrc_class = GST_BASE_SRC_CLASS(klass);
  GstPushSrcClass * pushsrc_class = GST_PUSH_SRC_CLASS(klass);

  gobject_class->set_property = rosrawimagesrc_set_property;
  gobject_class->get_property = rosrawimagesrc_get_property;
  gobject_class->finalize = rosrawimagesrc_finalize;

  g_object_class_install_property(
    gobject_class,
    PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic",
      "ROS Topic",
      "ROS topic to subscribe to",
      "image",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    gobject_class,
    PROP_ROS_RELIABLE,
    g_param_spec_boolean(
      "ros-reliable",
      "Reliable QoS",
      "Use RELIABLE QoS for the ROS subscription",
      FALSE,
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_add_static_pad_template(element_class, &src_template);
  gst_element_class_set_static_metadata(
    element_class,
    "ROS 2 Raw Image Source",
    "Source/Video",
    "Reads sensor_msgs/msg/Image from a ROS 2 topic",
    "Polymath Robotics <engineering@polymathrobotics.com>");

  element_class->change_state = rosrawimagesrc_change_state;
  basesrc_class->get_caps = rosrawimagesrc_getcaps;
  basesrc_class->unlock = rosrawimagesrc_unlock;
  basesrc_class->unlock_stop = rosrawimagesrc_unlock_stop;
  pushsrc_class->create = rosrawimagesrc_create;
}
