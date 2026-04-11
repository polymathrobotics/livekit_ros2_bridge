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

#include <mutex>
#include <string>

#include "gtest/gtest.h"
#include "utils/gstreamer_raii.hpp"

namespace livekit_ros2_bridge
{

namespace
{

void ensureGstreamerInitializedForTest()
{
  static std::once_flag once;
  std::call_once(once, []() {
    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
    }
  });
}

}  // namespace

TEST(GstreamerRaiiTest, IteratorAndValueGuardTraverseParsedPipeline)
{
  ensureGstreamerInitializedForTest();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch("fakesrc num-buffers=1 ! queue ! fakesink", &raw_error));
  GErrorPtr error(raw_error);

  ASSERT_NE(pipeline.get(), nullptr)
    << (error != nullptr && error->message != nullptr ? error->message : "parse failed");
  ASSERT_TRUE(GST_IS_BIN(pipeline.get()));

  GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline.get())));
  ASSERT_NE(iterator.get(), nullptr);

  GValueGuard item;
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
    EXPECT_NE(std::string(GST_ELEMENT_NAME(element)).size(), 0U);
    ++element_count;
    item.reset();
  }

  EXPECT_GT(element_count, 0U);
}

TEST(GstreamerRaiiTest, MapGuardSupportsWriteThenRead)
{
  ensureGstreamerInitializedForTest();

  GstBufferPtr buffer(gst_buffer_new_allocate(nullptr, 4U, nullptr));
  ASSERT_NE(buffer.get(), nullptr);

  {
    GstMapGuard map(buffer.get(), GST_MAP_WRITE);
    ASSERT_TRUE(map.is_valid());
    map.get()->data[0] = 1U;
    map.get()->data[1] = 2U;
    map.get()->data[2] = 3U;
    map.get()->data[3] = 4U;
  }

  {
    GstMapGuard map(buffer.get(), GST_MAP_READ);
    ASSERT_TRUE(map.is_valid());
    EXPECT_EQ(map.get()->size, 4U);
    EXPECT_EQ(map.get()->data[0], 1U);
    EXPECT_EQ(map.get()->data[1], 2U);
    EXPECT_EQ(map.get()->data[2], 3U);
    EXPECT_EQ(map.get()->data[3], 4U);
  }
}

TEST(GstreamerRaiiTest, ErrorAndStringWrappersAdoptAllocatedValues)
{
  ensureGstreamerInitializedForTest();

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
  ASSERT_NE(debug.get(), nullptr);
  EXPECT_STREQ(error->message, "synthetic failure");
  EXPECT_STREQ(debug.get(), "debug details");
}

}  // namespace livekit_ros2_bridge
