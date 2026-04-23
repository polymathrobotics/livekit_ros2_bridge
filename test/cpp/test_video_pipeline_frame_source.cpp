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

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "utils/gstreamer_raii.hpp"
#include "video_frame_source/ros_topic_video_frame_sources.hpp"
#include "video_frame_source/video_pipeline_frame_source.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

namespace
{

VideoStreamSpec makeTestSpec()
{
  VideoStreamSpec spec;
  spec.stream_key = "other_video:test";
  spec.track_name = "lkros.video.other.test";
  spec.input = OtherVideoInput{"test", "", ""};
  return spec;
}

VideoStreamSpec makeRosTopicSpec(const std::string & topic, const char * interface_type)
{
  VideoStreamSpec spec;
  spec.stream_key = "topic:" + topic;
  spec.track_name = "lkros.video.test";
  spec.input = RosVideoInput{topic, interface_type, classifyRosVideoIngestMode(interface_type).value(), "test", ""};
  return spec;
}

class NoOpFrameSink final : public VideoFrameSink
{
public:
  void write(int, int, std::vector<std::uint8_t>, std::int64_t) override
  {}
};

class NoOpLifecycleObserver final : public VideoStreamLifecycleObserver
{
public:
  void onSampleUnpackFailed(const std::string &) override
  {}

  void onCaptureFailed(const std::string &) override
  {}

  void onRestartFailed(const std::string &) override
  {}

  void onPushFailed(const std::string &) override
  {}
};

class TestableVideoPipelineFrameSource final : public VideoPipelineFrameSource
{
public:
  TestableVideoPipelineFrameSource(
    VideoFrameSink & sink,
    VideoStreamLifecycleObserver & observer,
    std::optional<RestartConfig> restart_config = std::nullopt)
  : VideoPipelineFrameSource(makeTestSpec(), sink, observer, std::move(restart_config))
  {}

  ~TestableVideoPipelineFrameSource() override
  {
    close();
  }

  void startPipeline(const std::string & description, bool require_appsrc)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    startPipelineLocked(description, require_appsrc);
  }

  bool hasAppSrc()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return appsrc_ != nullptr;
  }

  void simulateEos()
  {
    GstElement * pipeline = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pipeline_ == nullptr) {
        throw std::runtime_error("Test pipeline is not active.");
      }
      pipeline = pipeline_.get();
      gst_object_ref(pipeline);
    }

    GstMessage * message = gst_message_new_eos(GST_OBJECT(pipeline));
    gst_object_unref(pipeline);
    if (message == nullptr) {
      throw std::runtime_error("Failed to allocate test EOS message.");
    }

    onBusMessageThunk(nullptr, message, this);
    gst_message_unref(message);
  }
};

void expectStartErrorContains(
  const std::shared_ptr<TestableVideoPipelineFrameSource> & source,
  const std::string & description,
  bool require_appsrc,
  const char * expected_error_fragment)
{
  try {
    source->startPipeline(description, require_appsrc);
    FAIL() << "Expected startPipeline to throw an error containing '" << expected_error_fragment << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error_fragment), std::string::npos)
      << "actual error: " << error.what();
  }
}

class VideoPipelineFrameSourceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ensureGstreamerInitialized();
  }

  NoOpFrameSink sink_;
  NoOpLifecycleObserver observer_;
};

TEST_F(VideoPipelineFrameSourceTest, StartRejectsNamedNonAppSink)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  expectStartErrorContains(
    source, "videotestsrc is-live=true ! fakesink name=bridge_video_sink", false, "must be a GstAppSink");
}

TEST_F(VideoPipelineFrameSourceTest, StartRejectsNamedNonAppSrcWhenRequired)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  expectStartErrorContains(
    source,
    "videotestsrc is-live=true ! identity name=bridge_video_src ! appsink name=bridge_video_sink",
    true,
    "must be a GstAppSrc");
}

TEST_F(VideoPipelineFrameSourceTest, StartCapturesRequiredAppSrcHandle)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  source->startPipeline("appsrc name=bridge_video_src is-live=true ! appsink name=bridge_video_sink", true);

  EXPECT_TRUE(source->hasAppSrc());
  source->close();
}

TEST_F(VideoPipelineFrameSourceTest, OtherVideoLifecycleIsIdempotent)
{
  VideoStreamSpec spec = makeTestSpec();
  auto * input = otherVideoInput(spec);
  ASSERT_NE(input, nullptr);
  input->ingress_fragment = "videotestsrc is-live=true pattern=black";

  auto source = makeOtherVideoFrameSource(spec, sink_, observer_);
  source->close();
  source->close();
}

TEST_F(VideoPipelineFrameSourceTest, CloseCancelsDelayedRecoveryWithoutExtendingLifetime)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(
    sink_, observer_, VideoPipelineFrameSource::RestartConfig{false, std::chrono::seconds(5)});
  const std::weak_ptr<TestableVideoPipelineFrameSource> weak_source = source;

  source->startPipeline(
    "videotestsrc is-live=true ! videoconvert ! video/x-raw,format=I420 ! appsink name=bridge_video_sink", false);
  source->simulateEos();
  source->close();
  source.reset();

  EXPECT_TRUE(weak_source.expired());
}

void expectRosTopicSourceLifecycleIsIdempotent(
  VideoFrameSink & sink,
  VideoStreamLifecycleObserver & observer,
  const std::string & node_name,
  const std::string & topic,
  const char * interface_type)
{
  test_support::ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>(node_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto source = std::make_shared<RosTopicVideoFrameSource>(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    makeRosTopicSpec(topic, interface_type),
    nullptr,
    sink,
    observer);

  source->activate();
  source->activate();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 1U; }));

  source->close();
  source->close();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() { return node->count_subscribers(topic) == 0U; }));
}

TEST_F(VideoPipelineFrameSourceTest, RawRosTopicSourceLifecycleIsIdempotent)
{
  expectRosTopicSourceLifecycleIsIdempotent(
    sink_, observer_, "video_pipeline_ros_topic_raw_source_test", "/video_pipeline/raw_image", kImageInterfaceType);
}

TEST_F(VideoPipelineFrameSourceTest, CompressedRosTopicSourceLifecycleIsIdempotent)
{
  expectRosTopicSourceLifecycleIsIdempotent(
    sink_,
    observer_,
    "video_pipeline_ros_topic_compressed_source_test",
    "/video_pipeline/compressed_image",
    kCompressedImageInterfaceType);
}

}  // namespace

}  // namespace livekit_ros2_bridge
