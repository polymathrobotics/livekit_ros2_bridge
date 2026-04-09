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

#include <gst/gst.h>

#ifndef PACKAGE
  #define PACKAGE "livekit_ros2_bridge"
#endif

#include "roscompressedimagesrc.h"
#include "rosrawimagesrc.h"

GST_DEBUG_CATEGORY_STATIC(rosbridge_plugin_debug);
#define GST_CAT_DEFAULT rosbridge_plugin_debug

static gboolean register_element_factory(GstPlugin * plugin, const gchar * factory_name, GType type)
{
  if (gst_element_register(plugin, factory_name, GST_RANK_NONE, type)) {
    return TRUE;
  }

  GST_ERROR(
    "event=plugin_init_failed resource=%s kind=element_factory "
    "reason=gst_element_register_returned_false",
    factory_name);
  return FALSE;
}

// These factory names are part of the plugin surface that downstream pipelines
// construct directly, so renaming them is a compatibility break.
static gboolean plugin_init(GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT(rosbridge_plugin_debug, "rosbridge", 0, "ROS 2 image source plugin registration");

  gboolean ok = TRUE;
  ok &= register_element_factory(plugin, "rosrawimagesrc", GST_TYPE_ROSRAWIMAGESRC);
  ok &= register_element_factory(plugin, "roscompressedimagesrc", GST_TYPE_ROSCOMPRESSEDIMAGESRC);
  return ok;
}

GST_PLUGIN_DEFINE(
  GST_VERSION_MAJOR,
  GST_VERSION_MINOR,
  rosbridge,
  "ROS 2 image source elements for livekit_ros2_bridge",
  plugin_init,
  "0.1.0",
  "Proprietary",
  "livekit_ros2_bridge",
  "https://github.com/polymathrobotics/livekit_ros2_bridge")
