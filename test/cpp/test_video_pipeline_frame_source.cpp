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
#include "video_frame_source/configured_source_video_frame_source.hpp"
#include "video_frame_source/video_pipeline_frame_source.hpp"

namespace livekit_ros2_bridge
{

namespace
{

VideoStreamSpec makeTestSpec()
{
  VideoStreamSpec spec;
  spec.stream_key = "configured_source:test";
  spec.track_name = "configured_source.test";
  spec.input_kind = VideoInputKind::ConfiguredSource;
  spec.ingest_mode = kConfiguredSourceIngestMode;
  spec.config_id = "test";
  spec.source_name = "test";
  return spec;
}

class NoOpFrameSink final : public VideoFrameSink
{
public:
  void write(int, int, std::vector<std::uint8_t>, std::int64_t) override
  {}
};

class RecordingLifecycleObserver final : public VideoStreamLifecycleObserver
{
public:
  void onTrackPublished(int, int, bool) override
  {}

  void onTrackUnpublish() override
  {}

  void onSampleUnpackFailed(const std::string & error) override
  {
    sample_unpack_failures.push_back(error);
  }

  void onCaptureFailed(const std::string & error) override
  {
    capture_failures.push_back(error);
  }

  void onPipelineFailed(const std::string & reason) override
  {
    pipeline_failures.push_back(reason);
  }

  void onRestartFailed(const std::string & error) override
  {
    restart_failures.push_back(error);
  }

  void onPushFailed(const std::string & error) override
  {
    push_failures.push_back(error);
  }

  std::vector<std::string> sample_unpack_failures;
  std::vector<std::string> capture_failures;
  std::vector<std::string> pipeline_failures;
  std::vector<std::string> restart_failures;
  std::vector<std::string> push_failures;
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

class VideoPipelineFrameSourceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ensureGstreamerInitialized();
  }

  NoOpFrameSink sink_;
  RecordingLifecycleObserver observer_;
};

TEST_F(VideoPipelineFrameSourceTest, StartRejectsNamedNonAppSink)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  try {
    source->startPipeline("videotestsrc is-live=true ! fakesink name=bridge_video_sink", false);
    FAIL() << "expected start to reject a non-appsink element";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("must be a GstAppSink"), std::string::npos);
  }
}

TEST_F(VideoPipelineFrameSourceTest, StartRejectsNamedNonAppSrcWhenRequired)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  try {
    source->startPipeline(
      "videotestsrc is-live=true ! identity name=bridge_video_src ! appsink name=bridge_video_sink", true);
    FAIL() << "expected start to reject a non-appsrc element";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("must be a GstAppSrc"), std::string::npos);
  }
}

TEST_F(VideoPipelineFrameSourceTest, StartCapturesRequiredAppSrcHandle)
{
  auto source = std::make_shared<TestableVideoPipelineFrameSource>(sink_, observer_);

  source->startPipeline("appsrc name=bridge_video_src is-live=true ! appsink name=bridge_video_sink", true);

  EXPECT_TRUE(source->hasAppSrc());
  source->shutdown();
}

TEST_F(VideoPipelineFrameSourceTest, ConfiguredSourceLifecycleIsIdempotent)
{
  VideoStreamSpec spec = makeTestSpec();
  spec.ingress_fragment = "videotestsrc is-live=true pattern=black";

  auto source = std::make_shared<ConfiguredSourceVideoFrameSource>(spec, sink_, observer_);
  source->start();
  source->start();
  source->shutdown();
  source->shutdown();
}

}  // namespace

}  // namespace livekit_ros2_bridge
