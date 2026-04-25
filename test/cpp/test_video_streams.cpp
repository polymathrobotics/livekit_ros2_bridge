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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "fake_room_connection.hpp"
#include "gstreamer_pipeline.hpp"
#include "gstreamer_video_stream.hpp"
#include "gtest/gtest.h"
#include "livekit/video_frame.h"
#include "ros_test_support.hpp"
#include "ros_video_stream.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/gstreamer_raii.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

VideoStreamSpec makeOtherVideoSpec()
{
  VideoStreamSpec spec;
  spec.stream_key = "other_video:test";
  spec.track_name = "lkros.video.other.test";
  spec.input = OtherVideoInput{"test", "", ""};
  return spec;
}

VideoStreamSpec makeRosTopicSpec(const std::string & topic, const char * interface_type)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  config.ros_topic_rules.front().rule_id = "test";
  VideoStreamSpec spec = resolveRosVideoTopicSpec(config, topic, interface_type);
  spec.track_name = "lkros.video.test";
  return spec;
}

const char * imageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::Image>();
}

const char * compressedImageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>();
}

GStreamerPipelineCallbacks makeNoOpPipelineCallbacks()
{
  return GStreamerPipelineCallbacks{
    []() { return false; },
    [](const livekit::VideoFrame &, std::int64_t) {},
    [](const std::string &) {},
    [](const std::string &) {},
    [](const std::string &) {},
  };
}

void expectStartErrorContains(
  GStreamerPipeline & pipeline,
  const std::string & description,
  bool require_appsrc,
  const char * expected_error_fragment)
{
  try {
    pipeline.start(description, require_appsrc);
    FAIL() << "Expected startPipeline to throw an error containing '" << expected_error_fragment << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error_fragment), std::string::npos)
      << "actual error: " << error.what();
  }
}

class VideoStreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ensureGstreamerInitialized();
    static test_support::ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(VideoStreamTest, PipelineStartRejectsNamedNonAppSink)
{
  GStreamerPipeline pipeline(makeOtherVideoSpec(), makeNoOpPipelineCallbacks());

  expectStartErrorContains(
    pipeline, "videotestsrc is-live=true ! fakesink name=bridge_video_sink", false, "must be a GstAppSink");
}

TEST_F(VideoStreamTest, PipelineStartRejectsNamedNonAppSrcWhenRequired)
{
  GStreamerPipeline pipeline(makeOtherVideoSpec(), makeNoOpPipelineCallbacks());

  expectStartErrorContains(
    pipeline,
    "videotestsrc is-live=true ! identity name=bridge_video_src ! appsink name=bridge_video_sink",
    true,
    "must be a GstAppSrc");
}

TEST_F(VideoStreamTest, PipelineStartCapturesRequiredAppSrcHandle)
{
  GStreamerPipeline pipeline(makeOtherVideoSpec(), makeNoOpPipelineCallbacks());

  pipeline.start("appsrc name=bridge_video_src is-live=true ! appsink name=bridge_video_sink", true);

  EXPECT_NE(pipeline.appsrc(), nullptr);
  pipeline.stop();
}

TEST_F(VideoStreamTest, OtherVideoLifecycleIsIdempotent)
{
  VideoStreamSpec spec = makeOtherVideoSpec();
  spec.input = OtherVideoInput{"test", "videotestsrc is-live=true pattern=black", ""};

  FakeRoomConnection connection;
  VideoTrackPublisher publisher(connection, spec);
  GStreamerVideoStream stream(spec, publisher);

  stream.start();
  stream.close();
  stream.close();
}

void expectRosTopicStreamLifecycleIsIdempotent(
  const std::string & node_name, const std::string & topic, const char * interface_type)
{
  auto node = std::make_shared<rclcpp::Node>(node_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  VideoStreamSpec spec = makeRosTopicSpec(topic, interface_type);
  FakeRoomConnection connection;
  VideoTrackPublisher publisher(connection, spec);
  auto stream = std::make_shared<RosVideoStream>(*node, spec, nullptr, publisher);

  stream->start();
  stream->start();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 1U; }));

  stream->close();
  stream->close();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 0U; }));
}

TEST_F(VideoStreamTest, RawRosTopicStreamLifecycleIsIdempotent)
{
  expectRosTopicStreamLifecycleIsIdempotent(
    "video_pipeline_ros_topic_raw_source_test", "/video_pipeline/raw_image", imageInterfaceType());
}

TEST_F(VideoStreamTest, CompressedRosTopicStreamLifecycleIsIdempotent)
{
  expectRosTopicStreamLifecycleIsIdempotent(
    "video_pipeline_ros_topic_compressed_source_test",
    "/video_pipeline/compressed_image",
    compressedImageInterfaceType());
}

}  // namespace

}  // namespace livekit_ros2_bridge
