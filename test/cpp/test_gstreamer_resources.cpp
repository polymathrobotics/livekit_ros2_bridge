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

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "video/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::video
{

namespace
{

class GStreamerResourcesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ensureGStreamerInitialized();
  }
};

}  // namespace

TEST(GStreamerInitializationTest, EnsureGStreamerInitializedIsSafeAcrossConcurrentCalls)
{
  constexpr std::size_t kThreadCount = 8U;
  constexpr std::size_t kCallsPerThread = 32U;

  std::atomic<std::size_t> ready_count{0U};
  std::atomic<bool> start{false};
  std::atomic<bool> all_threads_observed_initialized{true};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back([&ready_count, &start, &all_threads_observed_initialized, kCallsPerThread]() {
      ready_count.fetch_add(1U, std::memory_order_relaxed);
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      for (std::size_t call = 0; call < kCallsPerThread; ++call) {
        ensureGStreamerInitialized();
        if (!gst_is_initialized()) {
          all_threads_observed_initialized.store(false, std::memory_order_relaxed);
        }
      }
    });
  }

  while (ready_count.load(std::memory_order_acquire) != kThreadCount) {
    std::this_thread::yield();
  }
  start.store(true, std::memory_order_release);

  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_TRUE(all_threads_observed_initialized.load(std::memory_order_relaxed));
}

TEST_F(GStreamerResourcesTest, IteratorAndValueSlotTraverseParsedPipeline)
{
  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch("fakesrc num-buffers=1 ! queue ! fakesink", &raw_error));
  GErrorPtr error(raw_error);

  ASSERT_NE(pipeline.get(), nullptr)
    << (error != nullptr && error->message != nullptr ? error->message : "parse failed");

  GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline.get())));
  ASSERT_NE(iterator.get(), nullptr);

  GValueSlot item;
  guint element_count = 0;
  while (true) {
    const GstIteratorResult result = gst_iterator_next(iterator.get(), item.get());
    if (result == GST_ITERATOR_DONE) {
      break;
    }
    if (result == GST_ITERATOR_RESYNC) {
      gst_iterator_resync(iterator.get());
      continue;
    }

    ASSERT_EQ(result, GST_ITERATOR_OK);
    auto * element = GST_ELEMENT(g_value_get_object(item.get()));
    ASSERT_NE(element, nullptr);
    ++element_count;
    item.reset();
  }

  EXPECT_GT(element_count, 0U);
}

TEST_F(GStreamerResourcesTest, BufferMapSupportsWriteThenRead)
{
  GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, 4U, nullptr));
  ASSERT_NE(buffer.get(), nullptr);

  const std::vector<guint8> expected_bytes{1U, 2U, 3U, 4U};

  {
    GstBufferMap map(*buffer, GST_MAP_WRITE);
    ASSERT_TRUE(map.is_valid());
    std::copy(expected_bytes.begin(), expected_bytes.end(), map.get()->data);
  }

  {
    GstBufferMap map(*buffer, GST_MAP_READ);
    ASSERT_TRUE(map.is_valid());
    const std::vector<guint8> actual_bytes(map.get()->data, map.get()->data + map.get()->size);
    EXPECT_EQ(actual_bytes, expected_bytes);
  }
}

TEST_F(GStreamerResourcesTest, VideoFrameMapSupportsWriteThenRead)
{
  GstVideoInfo info;
  gst_video_info_init(&info);
  ASSERT_TRUE(gst_video_info_set_format(&info, GST_VIDEO_FORMAT_RGB, 2U, 2U));

  GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, info.size, nullptr));
  ASSERT_NE(buffer.get(), nullptr);

  const std::vector<guint8> expected_bytes{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U};
  const guint row_bytes = 2U * 3U;

  {
    GstVideoFrameMap frame(info, *buffer, GST_MAP_WRITE);
    ASSERT_TRUE(frame.is_valid());

    auto * data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame.get(), 0));
    ASSERT_NE(data, nullptr);

    const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame.get(), 0);

    for (guint row = 0; row < 2U; ++row) {
      std::copy_n(expected_bytes.begin() + (row * row_bytes), row_bytes, data + (row * stride));
    }
  }

  {
    GstVideoFrameMap frame(info, *buffer, GST_MAP_READ);
    ASSERT_TRUE(frame.is_valid());

    const auto * data = static_cast<const guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame.get(), 0));
    ASSERT_NE(data, nullptr);

    const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame.get(), 0);
    std::vector<guint8> actual_bytes;
    actual_bytes.reserve(expected_bytes.size());
    for (guint row = 0; row < 2U; ++row) {
      actual_bytes.insert(actual_bytes.end(), data + (row * stride), data + (row * stride) + row_bytes);
    }

    EXPECT_EQ(actual_bytes, expected_bytes);
  }
}

TEST_F(GStreamerResourcesTest, VideoFrameMapRejectsUndersizedBuffer)
{
  GstVideoInfo info;
  gst_video_info_init(&info);
  ASSERT_TRUE(gst_video_info_set_format(&info, GST_VIDEO_FORMAT_RGB, 2U, 2U));

  GstBufferPtr undersized_buffer(gst_buffer_new_allocate(nullptr, info.size - static_cast<gsize>(1U), nullptr));
  ASSERT_NE(undersized_buffer.get(), nullptr);

  GstVideoFrameMap frame(info, *undersized_buffer, GST_MAP_READ);
  EXPECT_FALSE(frame.is_valid());
}

TEST_F(GStreamerResourcesTest, ErrorAndStringWrappersAdoptAllocatedValues)
{
  GstElementPtr source(gst_pipeline_new("test_source"));
  ASSERT_NE(source.get(), nullptr);

  GstMessage * message = gst_message_new_error(
    GST_OBJECT(source.get()),
    g_error_new_literal(g_quark_from_static_string("livekit_ros2_bridge.tests"), 7, "synthetic failure"),
    g_strdup("debug details"));
  ASSERT_NE(message, nullptr);

  GError * raw_error = nullptr;
  gchar * raw_debug = nullptr;
  gst_message_parse_error(message, &raw_error, &raw_debug);
  gst_message_unref(message);

  GErrorPtr error(raw_error);
  GCharPtr debug(raw_debug);
  ASSERT_NE(error.get(), nullptr);
  EXPECT_STREQ(error->message, "synthetic failure");
  EXPECT_STREQ(debug.get(), "debug details");
}

}  // namespace livekit_ros2_bridge::video
