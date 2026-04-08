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

#include "roscompressedimagesrc.h"

#include <algorithm>
#include <cctype>

GST_DEBUG_CATEGORY_STATIC(roscompressedimagesrc_debug);
#define GST_CAT_DEFAULT roscompressedimagesrc_debug

enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_RELIABLE,
};

static GstStaticPadTemplate src_template =
  GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("image/jpeg; image/png"));

// Parse the primary format from CompressedImage.format (e.g. "jpeg", "jpeg; jpeg_quality=95", "png")
static std::string parseCompressedFormat(const std::string & format_field)
{
  std::string primary = format_field;
  auto semi = primary.find(';');
  if (semi != std::string::npos) {
    primary = primary.substr(0, semi);
  }
  // Trim whitespace
  primary.erase(primary.begin(), std::find_if(primary.begin(), primary.end(), [](unsigned char c) {
                  return std::isspace(c) == 0;
                }));
  primary.erase(
    std::find_if(primary.rbegin(), primary.rend(), [](unsigned char c) { return std::isspace(c) == 0; }).base(),
    primary.end());
  // Lowercase
  std::transform(primary.begin(), primary.end(), primary.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return primary;
}

G_DEFINE_TYPE_WITH_CODE(
  RosCompressedImageSrc,
  roscompressedimagesrc,
  GST_TYPE_PUSH_SRC,
  GST_DEBUG_CATEGORY_INIT(roscompressedimagesrc_debug, "roscompressedimagesrc", 0, "ROS 2 compressed image source"))

static void roscompressedimagesrc_set_property(
  GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(object);
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

static void roscompressedimagesrc_get_property(GObject * object, guint prop_id, GValue * value, GParamSpec * pspec)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(object);
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

static void roscompressedimagesrc_finalize(GObject * object)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(object);
  g_free(self->ros_topic);
  self->sub.reset();
  self->node_runner.reset();
  self->detected_format.~basic_string();
  self->msg_queue.~deque();
  self->msg_queue_mtx.~mutex();
  self->msg_queue_cv.~condition_variable();
  G_OBJECT_CLASS(roscompressedimagesrc_parent_class)->finalize(object);
}

static sensor_msgs::msg::CompressedImage::ConstSharedPtr wait_for_compressed_msg(RosCompressedImageSrc * self)
{
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->msg_queue_cv.wait(lk, [self] { return !self->msg_queue.empty() || self->flushing; });
  if (self->flushing || self->msg_queue.empty()) {
    return nullptr;
  }
  return self->msg_queue.front();
}

static gboolean roscompressedimagesrc_unlock(GstBaseSrc * base_src)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->flushing = TRUE;
  self->msg_queue_cv.notify_all();
  return TRUE;
}

static gboolean roscompressedimagesrc_unlock_stop(GstBaseSrc * base_src)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
  self->flushing = FALSE;
  return TRUE;
}

static GstCaps * roscompressedimagesrc_getcaps(GstBaseSrc * base_src, GstCaps * filter)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(base_src);

  if (!self->caps_set) {
    if (!self->node_runner) {
      return gst_pad_get_pad_template_caps(base_src->srcpad);
    }

    GST_DEBUG_OBJECT(self, "waiting for first message to determine caps");
    auto msg = wait_for_compressed_msg(self);
    if (!msg) {
      return gst_pad_get_pad_template_caps(base_src->srcpad);
    }

    self->detected_format = parseCompressedFormat(msg->format);
    self->caps_set = TRUE;

    {
      std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
      self->msg_queue.clear();
    }
  }

  GstCaps * caps = NULL;
  if (self->detected_format == "jpeg" || self->detected_format == "jpg") {
    caps = gst_caps_new_empty_simple("image/jpeg");
  } else if (self->detected_format == "png") {
    caps = gst_caps_new_empty_simple("image/png");
  } else {
    GST_ERROR_OBJECT(self, "Unsupported CompressedImage format: %s", self->detected_format.c_str());
    return gst_pad_get_pad_template_caps(base_src->srcpad);
  }

  if (filter) {
    GstCaps * intersection = gst_caps_intersect_full(filter, caps, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref(caps);
    return intersection;
  }
  return caps;
}

static GstStateChangeReturn roscompressedimagesrc_change_state(GstElement * element, GstStateChange transition)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(element);

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY: {
      try {
        self->node_runner = std::make_unique<RosNodeRunner>("gst_compressed_image_src");
      } catch (const std::exception & e) {
        GST_ERROR_OBJECT(self, "Failed to create ROS node: %s", e.what());
        return GST_STATE_CHANGE_FAILURE;
      }

      auto cb = [self](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
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
      self->sub =
        self->node_runner->node()->create_subscription<sensor_msgs::msg::CompressedImage>(self->ros_topic, qos, cb);
      break;
    }

    default:
      break;
  }

  GstStateChangeReturn ret = GST_ELEMENT_CLASS(roscompressedimagesrc_parent_class)->change_state(element, transition);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    return ret;
  }

  switch (transition) {
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    case GST_STATE_CHANGE_READY_TO_NULL:
      self->sub.reset();
      self->node_runner.reset();
      self->caps_set = FALSE;
      self->detected_format.clear();
      break;

    default:
      break;
  }

  return ret;
}

static GstFlowReturn roscompressedimagesrc_create(GstPushSrc * push_src, GstBuffer ** buf)
{
  RosCompressedImageSrc * self = GST_ROSCOMPRESSEDIMAGESRC(push_src);

  auto msg = wait_for_compressed_msg(self);
  if (!msg) {
    return GST_FLOW_FLUSHING;
  }

  if (self->caps_set) {
    std::string msg_format = parseCompressedFormat(msg->format);
    if (msg_format != self->detected_format) {
      GST_WARNING_OBJECT(
        self, "CompressedImage format changed from '%s' to '%s'", self->detected_format.c_str(), msg_format.c_str());
      return GST_FLOW_ERROR;
    }
  }

  {
    std::unique_lock<std::mutex> lk(self->msg_queue_mtx);
    self->msg_queue.clear();
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

static void roscompressedimagesrc_init(RosCompressedImageSrc * self)
{
  self->ros_topic = g_strdup("image/compressed");
  self->ros_reliable = FALSE;
  self->flushing = FALSE;
  self->caps_set = FALSE;

  new (&self->detected_format) std::string();
  new (&self->msg_queue) std::deque<sensor_msgs::msg::CompressedImage::ConstSharedPtr>();
  new (&self->msg_queue_mtx) std::mutex();
  new (&self->msg_queue_cv) std::condition_variable();
  new (&self->node_runner) std::unique_ptr<RosNodeRunner>();
  new (&self->sub) rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr();

  gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp(GST_BASE_SRC(self), TRUE);
}

static void roscompressedimagesrc_class_init(RosCompressedImageSrcClass * klass)
{
  GObjectClass * gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass * basesrc_class = GST_BASE_SRC_CLASS(klass);
  GstPushSrcClass * pushsrc_class = GST_PUSH_SRC_CLASS(klass);

  gobject_class->set_property = roscompressedimagesrc_set_property;
  gobject_class->get_property = roscompressedimagesrc_get_property;
  gobject_class->finalize = roscompressedimagesrc_finalize;

  g_object_class_install_property(
    gobject_class,
    PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic",
      "ROS Topic",
      "ROS topic to subscribe to",
      "image/compressed",
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
    "ROS 2 Compressed Image Source",
    "Source/Video",
    "Reads sensor_msgs/msg/CompressedImage from a ROS 2 topic",
    "Polymath Robotics <engineering@polymathrobotics.com>");

  element_class->change_state = roscompressedimagesrc_change_state;
  basesrc_class->get_caps = roscompressedimagesrc_getcaps;
  basesrc_class->unlock = roscompressedimagesrc_unlock;
  basesrc_class->unlock_stop = roscompressedimagesrc_unlock_stop;
  pushsrc_class->create = roscompressedimagesrc_create;
}
