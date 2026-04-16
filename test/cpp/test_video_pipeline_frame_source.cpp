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
#include <vector>

#include "gtest/gtest.h"
#include "utils/gstreamer_raii.hpp"
#include "video_frame_source/other_video_frame_source.hpp"
#include "video_frame_source/video_pipeline_frame_source.hpp"

namespace livekit_ros2_bridge
{

namespace
{

VideoStreamSpec makeTestSpec()
{
  VideoStreamSpec spec;
  spec.stream_key = "other_video:test";
  spec.track_name = "ros.video.other.test";
  spec.input_kind = VideoInputKind::OtherVideoSource;
  spec.ingest_mode = kOtherVideoIngestMode;
  spec.config_id = "test";
  spec.other_video_source_name = "test";
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
  void onTrackPublished(int, int, bool) override
  {}

  void onTrackUnpublish() override
  {}

  void onSampleUnpackFailed(const std::string &) override
  {}

  void onCaptureFailed(const std::string &) override
  {}

  void onPipelineFailed(const std::string &) override
  {}

  void onRestartFailed(const std::string &) override
  {}

  void onPushFailed(const std::string &) override
  {}
};

class TestableVideoPipelineFrameSource final : public VideoPipelineFrameSource
{
public:
  TestableVideoPipelineFrameSource(VideoFrameSink & sink, VideoStreamLifecycleObserver & observer)
  : VideoPipelineFrameSource(makeTestSpec(), sink, observer)
  {}

  ~TestableVideoPipelineFrameSource() override
  {
    shutdown();
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
  source->shutdown();
}

TEST_F(VideoPipelineFrameSourceTest, OtherVideoLifecycleIsIdempotent)
{
  VideoStreamSpec spec = makeTestSpec();
  spec.ingress_fragment = "videotestsrc is-live=true pattern=black";

  auto source = std::make_shared<OtherVideoFrameSource>(spec, sink_, observer_);
  source->start();
  source->start();
  source->shutdown();
  source->shutdown();
}

}  // namespace

}  // namespace livekit_ros2_bridge
