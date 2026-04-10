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

#include <gst/base/gstbasesrc.h>
#include <gst/base/gstpushsrc.h>
#include <gst/gst.h>

#include <memory>

#include "encoding_utils.hpp"
#include "gtest/gtest.h"
#include "roscompressedimagesrc.h"
#include "rosrawimagesrc.h"

namespace
{

constexpr int kNegotiatedImageWidth = 640;
constexpr int kNegotiatedImageHeight = 480;
constexpr int kIncomingMismatchedImageWidth = 1280;
constexpr int kIncomingMismatchedImageHeight = 720;
constexpr int kRgbBytesPerPixel = 3;

class RosImageSrcTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    gst_init(nullptr, nullptr);
  }
};

struct GstElementUnref
{
  void operator()(GstElement * element) const
  {
    gst_object_unref(element);
  }
};

using GstElementPtr = std::unique_ptr<GstElement, GstElementUnref>;

template <typename SourceT>
struct SourceHarness
{
  GstElementPtr element;
  SourceT * instance;
  GstBaseSrcClass * base_src_class;
  GstPushSrcClass * push_src_class;
};

template <typename SourceT, typename CastFn>
SourceHarness<SourceT> makeSourceHarness(GType type, CastFn cast)
{
  GstElementPtr element{GST_ELEMENT(g_object_new(type, nullptr))};
  GstElement * raw_element = element.get();
  return {
    std::move(element),
    cast(raw_element),
    GST_BASE_SRC_GET_CLASS(raw_element),
    GST_PUSH_SRC_GET_CLASS(raw_element),
  };
}

template <typename SourceT, typename CastFn>
void expectUnlockCycleResetsFlushing(GType type, CastFn cast)
{
  auto source = makeSourceHarness<SourceT>(type, cast);

  ASSERT_NE(source.base_src_class->unlock, nullptr);
  ASSERT_NE(source.base_src_class->unlock_stop, nullptr);
  ASSERT_TRUE(source.base_src_class->unlock(GST_BASE_SRC(source.element.get())));
  ASSERT_TRUE(source.instance->flushing);
  EXPECT_TRUE(source.base_src_class->unlock_stop(GST_BASE_SRC(source.element.get())));
  EXPECT_FALSE(source.instance->flushing);
}

template <typename SourceT, typename CastFn>
void expectCreateReturnsFlushingAfterUnlock(GType type, CastFn cast)
{
  auto source = makeSourceHarness<SourceT>(type, cast);

  ASSERT_NE(source.base_src_class->unlock, nullptr);
  ASSERT_NE(source.push_src_class->create, nullptr);
  ASSERT_TRUE(source.base_src_class->unlock(GST_BASE_SRC(source.element.get())));

  GstBuffer * buffer = nullptr;
  EXPECT_EQ(source.push_src_class->create(GST_PUSH_SRC(source.element.get()), &buffer), GST_FLOW_FLUSHING);
  EXPECT_EQ(buffer, nullptr);
}

// --- RosRawImageSrc ---

TEST_F(RosImageSrcTest, RawImageSrcUnlockStopClearsFlushing)
{
  expectUnlockCycleResetsFlushing<RosRawImageSrc>(
    rosrawimagesrc_get_type(), [](GstElement * element) { return GST_ROSRAWIMAGESRC(element); });
}

TEST_F(RosImageSrcTest, RawImageSrcCreateReturnsFlushingAfterUnlock)
{
  expectCreateReturnsFlushingAfterUnlock<RosRawImageSrc>(
    rosrawimagesrc_get_type(), [](GstElement * element) { return GST_ROSRAWIMAGESRC(element); });
}

TEST_F(RosImageSrcTest, RawImageSrcCreateReturnsErrorOnDimensionMismatch)
{
  auto source = makeSourceHarness<RosRawImageSrc>(
    rosrawimagesrc_get_type(), [](GstElement * element) { return GST_ROSRAWIMAGESRC(element); });

  source.instance->caps_set = TRUE;
  source.instance->width = kNegotiatedImageWidth;
  source.instance->height = kNegotiatedImageHeight;
  source.instance->format = GST_VIDEO_FORMAT_RGB;

  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->width = kIncomingMismatchedImageWidth;
  message->height = kIncomingMismatchedImageHeight;
  message->encoding = "rgb8";
  message->data.resize(kIncomingMismatchedImageWidth * kIncomingMismatchedImageHeight * kRgbBytesPerPixel);

  {
    std::unique_lock<std::mutex> lk(source.instance->msg_queue_mtx);
    source.instance->msg_queue.push_front(message);
  }

  GstBuffer * buffer = nullptr;
  EXPECT_EQ(source.push_src_class->create(GST_PUSH_SRC(source.element.get()), &buffer), GST_FLOW_ERROR);
  EXPECT_EQ(buffer, nullptr);
}

TEST_F(RosImageSrcTest, RawImageSrcCreateReturnsErrorOnEncodingMismatch)
{
  auto source = makeSourceHarness<RosRawImageSrc>(
    rosrawimagesrc_get_type(), [](GstElement * element) { return GST_ROSRAWIMAGESRC(element); });

  source.instance->caps_set = TRUE;
  source.instance->width = kNegotiatedImageWidth;
  source.instance->height = kNegotiatedImageHeight;
  source.instance->format = GST_VIDEO_FORMAT_RGB;

  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->width = kNegotiatedImageWidth;
  message->height = kNegotiatedImageHeight;
  message->encoding = "bgr8";
  message->data.resize(kNegotiatedImageWidth * kNegotiatedImageHeight * kRgbBytesPerPixel);

  {
    std::unique_lock<std::mutex> lk(source.instance->msg_queue_mtx);
    source.instance->msg_queue.push_front(message);
  }

  GstBuffer * buffer = nullptr;
  EXPECT_EQ(source.push_src_class->create(GST_PUSH_SRC(source.element.get()), &buffer), GST_FLOW_ERROR);
  EXPECT_EQ(buffer, nullptr);
}

TEST_F(RosImageSrcTest, RosEncodingToGstFormatReturnsExactSupportedMappings)
{
  struct EncodingCase
  {
    const char * encoding;
    GstVideoFormat format;
  };

  const EncodingCase cases[] = {
    {"mono8", GST_VIDEO_FORMAT_GRAY8},
    {"mono16", GST_VIDEO_FORMAT_GRAY16_LE},
    {"rgb8", GST_VIDEO_FORMAT_RGB},
    {"bgr8", GST_VIDEO_FORMAT_BGR},
    {"rgba8", GST_VIDEO_FORMAT_RGBA},
    {"bgra8", GST_VIDEO_FORMAT_BGRA},
    {"yuv422", GST_VIDEO_FORMAT_UYVY},
    {"yuv422_yuy2", GST_VIDEO_FORMAT_YUY2},
  };

  for (const auto & test_case : cases) {
    EXPECT_EQ(rosEncodingToGstFormat(test_case.encoding), test_case.format) << test_case.encoding;
  }
}

TEST_F(RosImageSrcTest, RosEncodingToGstFormatReturnsUnknownForUnsupportedEncodings)
{
  for (const auto * encoding : {"bayer_rggb8", "bayer_grbg8", "", "32FC1"}) {
    EXPECT_EQ(rosEncodingToGstFormat(encoding), GST_VIDEO_FORMAT_UNKNOWN) << encoding;
  }
}

// --- RosCompressedImageSrc ---

TEST_F(RosImageSrcTest, CompressedImageSrcUnlockStopClearsFlushing)
{
  expectUnlockCycleResetsFlushing<RosCompressedImageSrc>(
    roscompressedimagesrc_get_type(), [](GstElement * element) { return GST_ROSCOMPRESSEDIMAGESRC(element); });
}

TEST_F(RosImageSrcTest, CompressedImageSrcCreateReturnsFlushingAfterUnlock)
{
  expectCreateReturnsFlushingAfterUnlock<RosCompressedImageSrc>(
    roscompressedimagesrc_get_type(), [](GstElement * element) { return GST_ROSCOMPRESSEDIMAGESRC(element); });
}

TEST_F(RosImageSrcTest, CompressedImageSrcCreateReturnsErrorOnFormatMismatch)
{
  auto source = makeSourceHarness<RosCompressedImageSrc>(
    roscompressedimagesrc_get_type(), [](GstElement * element) { return GST_ROSCOMPRESSEDIMAGESRC(element); });

  source.instance->caps_set = TRUE;
  source.instance->detected_format = "jpeg";

  auto message = std::make_shared<sensor_msgs::msg::CompressedImage>();
  message->format = "png";
  message->data = {0x89, 0x50, 0x4E, 0x47};

  {
    std::unique_lock<std::mutex> lk(source.instance->msg_queue_mtx);
    source.instance->msg_queue.push_front(message);
  }

  GstBuffer * buffer = nullptr;
  EXPECT_EQ(source.push_src_class->create(GST_PUSH_SRC(source.element.get()), &buffer), GST_FLOW_ERROR);
  EXPECT_EQ(buffer, nullptr);
}

}  // namespace
