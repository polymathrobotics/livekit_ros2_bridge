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

// These factory names are part of the plugin surface that downstream pipelines
// construct directly, so renaming them is a compatibility break.
static gboolean plugin_init(GstPlugin * plugin)
{
  gboolean ok = TRUE;
  ok &= gst_element_register(plugin, "rosrawimagesrc", GST_RANK_NONE, GST_TYPE_ROSRAWIMAGESRC);
  ok &= gst_element_register(plugin, "roscompressedimagesrc", GST_RANK_NONE, GST_TYPE_ROSCOMPRESSEDIMAGESRC);
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
