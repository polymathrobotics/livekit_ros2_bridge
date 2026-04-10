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

#include "encoding_utils.hpp"

#include <sensor_msgs/image_encodings.hpp>

GstVideoFormat rosEncodingToGstFormat(const std::string & encoding)
{
  // Keep this intentionally limited to the raw formats exposed in
  // GST_ROS_VIDEO_FORMAT_LIST instead of guessing caps for other ROS encodings.
  if (encoding == sensor_msgs::image_encodings::MONO8) return GST_VIDEO_FORMAT_GRAY8;
  if (encoding == sensor_msgs::image_encodings::MONO16) return GST_VIDEO_FORMAT_GRAY16_LE;
  if (encoding == sensor_msgs::image_encodings::RGB8) return GST_VIDEO_FORMAT_RGB;
  if (encoding == sensor_msgs::image_encodings::BGR8) return GST_VIDEO_FORMAT_BGR;
  if (encoding == sensor_msgs::image_encodings::RGBA8) return GST_VIDEO_FORMAT_RGBA;
  if (encoding == sensor_msgs::image_encodings::BGRA8) return GST_VIDEO_FORMAT_BGRA;
  if (encoding == sensor_msgs::image_encodings::YUV422) return GST_VIDEO_FORMAT_UYVY;
  if (encoding == sensor_msgs::image_encodings::YUV422_YUY2) return GST_VIDEO_FORMAT_YUY2;
  return GST_VIDEO_FORMAT_UNKNOWN;
}
