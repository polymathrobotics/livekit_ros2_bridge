// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "video/gstreamer_pipeline.hpp"
#include "video/pipeline_description.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{
namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::VideoSource; see ScopedLiveKitInit.
const test_support::ScopedLiveKitInit kLiveKitInit;

// Mirror TrackPublisher's private intervals.
constexpr auto kKeyFrameRequestInterval = std::chrono::milliseconds(500);
constexpr auto kLogThrottle = std::chrono::seconds(5);
// Slack for waits that must clearly outlast an interval or let another thread block.
constexpr auto kTimingMargin = std::chrono::milliseconds(100);

StreamSpec makeSpec(const std::string & stream_key, const std::string & track_name)
{
  StreamSpec spec;
  spec.stream_key = stream_key;
  spec.track_name = track_name;
  return spec;
}

livekit::VideoFrame makeFrame(int width, int height)
{
  const auto luma_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const auto chroma_width = static_cast<std::size_t>((width + 1) / 2);
  const auto chroma_height = static_cast<std::size_t>((height + 1) / 2);
  const auto chroma_plane_size = chroma_width * chroma_height;
  return livekit::VideoFrame(
    width,
    height,
    livekit::VideoBufferType::I420,
    std::vector<std::uint8_t>(luma_size + (2U * chroma_plane_size), 128U));
}

StreamSpec makePassthroughSpec(const std::string & stream_key, const std::string & track_name)
{
  StreamSpec spec = makeSpec(stream_key, track_name);
  spec.publish_options.video_encoder = livekit::VideoEncoderBackend::PreEncoded;
  spec.publish_options.simulcast = false;
  return spec;
}

// The publisher forwards the bytes without reading them, so they are H.264 whatever `codec` says.
EncodedVideoFrame makeEncodedFrame(
  bool is_keyframe, std::uint32_t width, std::uint32_t height, livekit::VideoCodec codec = livekit::VideoCodec::H264)
{
  EncodedVideoFrame encoded;
  encoded.codec = codec;
  encoded.frame.is_keyframe = is_keyframe;
  encoded.frame.width = width;
  encoded.frame.height = height;
  encoded.frame.timestamp_us = 1000;
  // Annex-B start code and an IDR (type 5) or non-IDR (type 1) slice header.
  const std::uint8_t nal_header = is_keyframe ? 0x65U : 0x41U;
  encoded.frame.data = {0x00U, 0x00U, 0x00U, 0x01U, nal_header, 0x88U};
  return encoded;
}

// LiveKit rejects an empty frame, so sending this one fails.
EncodedVideoFrame makeEmptyDeltaFrame()
{
  EncodedVideoFrame encoded = makeEncodedFrame(false, 64, 48);
  encoded.frame.data.clear();
  return encoded;
}

std::shared_ptr<livekit::EncodedVideoSource> publishedEncodedSource(
  const FakeRoomConnection & connection, std::size_t index)
{
  return std::dynamic_pointer_cast<livekit::EncodedVideoSource>(connection.state->published_video_sources.at(index));
}

TEST(TrackPublisherTest, RepublishesOnSizeChangeAndUnpublishesOnDestruction)
{
  FakeRoomConnection connection;
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("stream:lifecycle", "lkros.video.camera.lifecycle"));

  publisher->capture(makeFrame(2, 2), 1000);
  publisher->capture(makeFrame(2, 2), 2000);
  publisher->capture(makeFrame(4, 4), 3000);
  publisher.reset();
  publisher.reset();

  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_video_track:lkros.video.camera.lifecycle",
      "unpublish_video_track:lkros.video.camera.lifecycle",
      "publish_video_track:lkros.video.camera.lifecycle",
      "unpublish_video_track:lkros.video.camera.lifecycle",
    }));
}

TEST(TrackPublisherTest, EnablesUserTimestampFrameMetadataForPublishedVideo)
{
  FakeRoomConnection connection;
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("stream:metadata", "lkros.video.camera.metadata"));

  publisher->capture(makeFrame(2, 2), 1000);

  ASSERT_EQ(connection.state->published_video_options.size(), 1U);
  ASSERT_TRUE(connection.state->published_video_options.front().frame_metadata_features.has_value());
  EXPECT_TRUE(connection.state->published_video_options.front().frame_metadata_features->user_timestamp);
  EXPECT_FALSE(connection.state->published_video_options.front().frame_metadata_features->frame_id);
}

TEST(TrackPublisherTest, DestructionUsesBestEffortPublishedTrackCleanup)
{
  FakeRoomConnection connection;
  connection.state->throw_on_unpublish_video = true;
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makeSpec("stream:unpublish_failure", "lkros.video.camera.unpublish_failure"));

  publisher->capture(makeFrame(2, 2), 1000);
  EXPECT_NO_THROW(publisher.reset());
  EXPECT_NO_THROW(publisher.reset());

  EXPECT_EQ(
    connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.camera.unpublish_failure"}));
  EXPECT_EQ(
    connection.state->unpublished_video_track_names,
    (std::vector<std::string>{"lkros.video.camera.unpublish_failure"}));
}

TEST(TrackPublisherTest, PublishFailureOnFirstFrameCanRetryAndStillDestroyCleanly)
{
  FakeRoomConnection connection;
  int publish_attempts = 0;
  connection.state->publish_video_track_hook = [&](const std::string &) {
    ++publish_attempts;
    if (publish_attempts == 1) {
      throw std::runtime_error("simulated video publish failure");
    }
  };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("stream:publish_retry", "lkros.video.camera.publish_retry"));

  EXPECT_THROW(publisher->capture(makeFrame(2, 2), 1000), std::runtime_error);
  publisher->capture(makeFrame(2, 2), 2000);
  publisher.reset();

  EXPECT_EQ(
    connection.state->published_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.publish_retry",
      "lkros.video.camera.publish_retry",
    }));
  EXPECT_EQ(
    connection.state->unpublished_video_track_names, (std::vector<std::string>{"lkros.video.camera.publish_retry"}));
}

TEST(TrackPublisherTest, RepublishFailureLeavesPublisherReadyForRetry)
{
  FakeRoomConnection connection;
  int publish_attempts = 0;
  connection.state->publish_video_track_hook = [&](const std::string &) {
    ++publish_attempts;
    if (publish_attempts == 2) {
      throw std::runtime_error("simulated video publish failure");
    }
  };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makeSpec("stream:republish_retry", "lkros.video.camera.republish_retry"));

  publisher->capture(makeFrame(2, 2), 1000);
  EXPECT_THROW(publisher->capture(makeFrame(4, 4), 2000), std::runtime_error);
  publisher->capture(makeFrame(4, 4), 3000);
  publisher.reset();

  EXPECT_EQ(
    connection.state->published_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
    }));
  EXPECT_EQ(
    connection.state->unpublished_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
    }));
}

TEST(TrackPublisherTest, BlockingPublishStillUnpublishesOnceOnDestruction)
{
  FakeRoomConnection connection;
  bool publish_started = false;
  std::promise<void> publish_started_promise;
  std::future<void> publish_started_future = publish_started_promise.get_future();
  std::promise<void> release_publish_promise;
  std::shared_future<void> release_publish_future = release_publish_promise.get_future().share();
  connection.state->publish_video_track_hook = [&](const std::string &) {
    if (!publish_started) {
      publish_started = true;
      publish_started_promise.set_value();
    }
    release_publish_future.wait();
  };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makeSpec("stream:concurrent_shutdown", "lkros.video.camera.concurrent_shutdown"));

  auto write_future = std::async(std::launch::async, [&publisher]() { publisher->capture(makeFrame(2, 2), 1000); });
  EXPECT_EQ(publish_started_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  release_publish_promise.set_value();

  EXPECT_EQ(write_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  write_future.get();
  publisher.reset();

  EXPECT_EQ(
    connection.state->published_video_track_names,
    (std::vector<std::string>{"lkros.video.camera.concurrent_shutdown"}));
  EXPECT_EQ(
    connection.state->unpublished_video_track_names,
    (std::vector<std::string>{"lkros.video.camera.concurrent_shutdown"}));
}

TEST(TrackPublisherTest, PassthroughPublishesEncodedSourceOnFirstKeyFrame)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:passthrough", "lkros.video.passthrough"));

  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 0U);
  // Delta frames before the first keyframe are discarded; the request is throttled.
  EXPECT_EQ(key_frame_requests, 1);

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_video_track:lkros.video.passthrough",
      "unpublish_video_track:lkros.video.passthrough",
    }));
  ASSERT_EQ(connection.state->published_video_options.size(), 1U);
  const auto & options = connection.state->published_video_options.front();
  ASSERT_TRUE(options.frame_metadata_features.has_value());
  EXPECT_TRUE(options.frame_metadata_features->user_timestamp);
  // Throttled: one line, logged at the first discarded frame.
  EXPECT_EQ(
    logs.fieldValues(
      {"video_stream_passthrough_waiting_for_key_frame", "stream_key=stream:passthrough"}, "discarded_frames"),
    std::vector<std::string>{"1"});
}

// The source's codec and the track's must match, or LiveKit rejects every frame.
TEST(TrackPublisherTest, PassthroughPublishesWithTheCodecOfTheFirstKeyFrame)
{
  for (const livekit::VideoCodec codec :
       {livekit::VideoCodec::H264, livekit::VideoCodec::H265, livekit::VideoCodec::AV1})
  {
    SCOPED_TRACE(static_cast<int>(codec));
    FakeRoomConnection connection;
    auto publisher =
      std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:codec", "lkros.video.codec"));

    publisher->captureEncoded(makeEncodedFrame(true, 64, 48, codec), []() {});
    publisher.reset();

    ASSERT_EQ(connection.state->published_video_options.size(), 1U);
    EXPECT_EQ(connection.state->published_video_options.front().video_codec, codec);
    const auto source = publishedEncodedSource(connection, 0U);
    ASSERT_NE(source, nullptr);
    EXPECT_EQ(source->codec(), codec);
  }
}

// A restarted source can send another codec. Its frames wait for its first keyframe, which
// republishes the track with that codec.
TEST(TrackPublisherTest, PassthroughRepublishesWhenTheCodecChanges)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:recodec", "lkros.video.recodec"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48, livekit::VideoCodec::H264), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48, livekit::VideoCodec::H264), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48, livekit::VideoCodec::H265), request_key_frame);
  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(connection.state->publishedVideoTrackCount(), 1U);

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48, livekit::VideoCodec::H265), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48, livekit::VideoCodec::H265), request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_video_track:lkros.video.recodec",
      "unpublish_video_track:lkros.video.recodec",
      "publish_video_track:lkros.video.recodec",
      "unpublish_video_track:lkros.video.recodec",
    }));
  ASSERT_EQ(connection.state->published_video_options.size(), 2U);
  EXPECT_EQ(connection.state->published_video_options.back().video_codec, livekit::VideoCodec::H265);
  const auto republished_source = publishedEncodedSource(connection, 1U);
  ASSERT_NE(republished_source, nullptr);
  EXPECT_EQ(republished_source->codec(), livekit::VideoCodec::H265);
  EXPECT_EQ(
    logs.countLines(
      {"video_stream_track_republished", "stream_key=stream:recodec", "previous_codec=h264", " codec=h265"}),
    1U);
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:recodec"}, "discarded_frames"),
    std::vector<std::string>{"1"});
}

TEST(TrackPublisherTest, PassthroughSizeChangeDoesNotRepublish)
{
  FakeRoomConnection connection;
  const std::function<void()> request_key_frame = []() {};
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:resize", "lkros.video.resize"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(true, 128, 96), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 128, 96), request_key_frame);
  publisher.reset();

  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_video_track:lkros.video.resize",
      "unpublish_video_track:lkros.video.resize",
    }));
}

TEST(TrackPublisherTest, PassthroughAwaitKeyFrameDiscardsUntilNextKeyFrame)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:restart", "lkros.video.restart"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  EXPECT_EQ(key_frame_requests, 0);

  publisher->awaitKeyFrame();
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  EXPECT_EQ(key_frame_requests, 1);

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.restart"}));
  EXPECT_EQ(connection.state->unpublished_video_track_names, (std::vector<std::string>{"lkros.video.restart"}));
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:restart"}, "discarded_frames"),
    std::vector<std::string>{"2"});
  // Only the first wait logs: later ones follow a logged restart or fall behind.
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_waiting_for_key_frame"}), 0U);
}

TEST(TrackPublisherTest, PassthroughFellBehindRequestsThrottledKeyFrameAndDiscardsUntilIt)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:behind", "lkros.video.behind"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  // Overruns right after the publish are its backlog draining.
  std::this_thread::sleep_for(kBridgeQueueDuration + kTimingMargin);
  publisher->onFellBehind(request_key_frame);
  EXPECT_EQ(key_frame_requests, 1);
  // The queue overruns again while it stays full; that must not ask again.
  publisher->onFellBehind(request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher->onFellBehind(request_key_frame);
  EXPECT_EQ(key_frame_requests, 1);

  // Once the keyframe arrives, the next fall behind asks again right away.
  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->onFellBehind(request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 2);
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:behind"}, "discarded_frames"),
    std::vector<std::string>{"1"});
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_fell_behind", "stream_key=stream:behind"}, "count"),
    std::vector<std::string>{"1"});
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.behind"}));
}

TEST(TrackPublisherTest, PassthroughIgnoresFellBehindWhileFirstKeyFramePublishes)
{
  FakeRoomConnection connection;
  std::promise<void> publish_started_promise;
  std::future<void> publish_started_future = publish_started_promise.get_future();
  std::promise<void> release_publish_promise;
  std::shared_future<void> release_publish_future = release_publish_promise.get_future().share();
  connection.state->publish_video_track_hook = [&](const std::string &) {
    publish_started_promise.set_value();
    (void)release_publish_future.wait_for(std::chrono::seconds(10));
  };
  std::atomic<int> key_frame_requests{0};
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makePassthroughSpec("stream:slow_publish", "lkros.video.slow_publish"));

  auto capture_future = std::async(
    std::launch::async, [&]() { publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame); });
  const bool publish_started = publish_started_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  auto fell_behind_future = std::async(std::launch::async, [&]() { publisher->onFellBehind(request_key_frame); });
  // Let onFellBehind() reach the publisher and wait for the publish to end.
  std::this_thread::sleep_for(kTimingMargin);
  release_publish_promise.set_value();
  capture_future.get();
  fell_behind_future.get();
  // Still live: the delta frame is sent, not discarded while waiting for a keyframe.
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher.reset();

  EXPECT_TRUE(publish_started);
  EXPECT_EQ(key_frame_requests.load(), 0);
}

TEST(TrackPublisherTest, PassthroughIgnoresFellBehindForOneQueueLengthAfterSlowPublish)
{
  FakeRoomConnection connection;
  // Longer than the bridge queue holds, so the backlog overruns it as it drains.
  connection.state->publish_video_track_hook = [](const std::string &) {
    std::this_thread::sleep_for(kBridgeQueueDuration + kTimingMargin);
  };
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makePassthroughSpec("stream:stalled_publish", "lkros.video.stalled_publish"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->onFellBehind(request_key_frame);
  EXPECT_EQ(key_frame_requests, 0);

  std::this_thread::sleep_for(kBridgeQueueDuration + kTimingMargin);
  publisher->onFellBehind(request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
}

TEST(TrackPublisherTest, PassthroughPublishFailureThrowsAndRetriesAtNextKeyFrame)
{
  FakeRoomConnection connection;
  int publish_attempts = 0;
  connection.state->publish_video_track_hook = [&](const std::string &) {
    ++publish_attempts;
    if (publish_attempts == 1) {
      throw std::runtime_error("simulated video publish failure");
    }
  };
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:retry", "lkros.video.retry"));

  // Thrown like a raw-frame failure, so the pipeline fails and restarts.
  EXPECT_THROW(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame), std::runtime_error);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  EXPECT_EQ(publish_attempts, 1);
  EXPECT_EQ(key_frame_requests, 1);

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher.reset();

  EXPECT_EQ(publish_attempts, 2);
  EXPECT_EQ(
    connection.state->published_video_track_names,
    (std::vector<std::string>{"lkros.video.retry", "lkros.video.retry"}));
  EXPECT_EQ(connection.state->unpublished_video_track_names, (std::vector<std::string>{"lkros.video.retry"}));
}

TEST(TrackPublisherTest, PassthroughSendFailureWaitsForNextKeyFrame)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:send", "lkros.video.send"));

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  // Logged, not thrown: a throw would fail the GStreamer pipeline and restart it.
  EXPECT_NO_THROW(publisher->captureEncoded(makeEmptyDeltaFrame(), request_key_frame));
  // The picture is broken until the next keyframe, so delta frames are discarded.
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  EXPECT_EQ(key_frame_requests, 1);

  publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
  publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.send"}));
  // Only the delta frame after the failure was discarded.
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:send"}, "discarded_frames"),
    std::vector<std::string>{"1"});
}

TEST(TrackPublisherTest, PassthroughIntermittentSendFailuresStayThrottled)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:send_storm", "lkros.video.send_storm"));
  constexpr int kFailedSends = 20;

  const auto started_at = std::chrono::steady_clock::now();
  // Each keyframe is sent, the next frame fails, and the one after is discarded while waiting.
  for (int failed_send = 0; failed_send < kFailedSends; ++failed_send) {
    publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
    publisher->captureEncoded(makeEmptyDeltaFrame(), request_key_frame);
    publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  }
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  publisher.reset();

  // The throttles bound the requests and warnings however long the loop took.
  EXPECT_GE(key_frame_requests, 1);
  EXPECT_LE(key_frame_requests, 1 + elapsed / kKeyFrameRequestInterval);
  const std::vector<std::string> counts =
    logs.fieldValues({"video_stream_capture_failed", "stream_key=stream:send_storm"}, "count");
  ASSERT_FALSE(counts.empty());
  EXPECT_EQ(counts.front(), "1");
  EXPECT_LE(counts.size(), 1U + static_cast<std::size_t>(elapsed / kLogThrottle));
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.send_storm"}));
}

TEST(PassthroughBitrateMonitorTest, ReportsOnlyAfterStayingWellAboveTarget)
{
  using std::chrono::milliseconds;
  PassthroughBitrateMonitor monitor;
  monitor.setTarget(1'000'000U);
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(100);

  // 25 kB every 100 ms is 2 Mbit/s, twice the target.
  std::optional<int> first_report_index;
  std::optional<std::uint64_t> first_report_bps;
  for (int frame_index = 0; frame_index <= 50; ++frame_index) {
    const auto reported = monitor.record(25'000U, start + milliseconds(100 * frame_index));
    if (reported.has_value() && !first_report_index.has_value()) {
      first_report_index = frame_index;
      first_report_bps = reported;
    }
  }

  ASSERT_TRUE(first_report_index.has_value());
  EXPECT_EQ(*first_report_index, 30);
  EXPECT_EQ(*first_report_bps, 2'000'000U);
}

TEST(PassthroughBitrateMonitorTest, StaysQuietWithinBudgetWithoutTargetOrWhenOverrunIsBrief)
{
  using std::chrono::milliseconds;
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(100);

  PassthroughBitrateMonitor without_target;
  PassthroughBitrateMonitor within_budget;
  within_budget.setTarget(1'000'000U);
  PassthroughBitrateMonitor brief_overrun;
  brief_overrun.setTarget(1'000'000U);
  for (int frame_index = 0; frame_index <= 100; ++frame_index) {
    const auto now = start + milliseconds(100 * frame_index);
    EXPECT_FALSE(without_target.record(25'000U, now).has_value());
    // 18.75 kB every 100 ms is 1.5 Mbit/s: exactly 1.5x the target is not well above it.
    EXPECT_FALSE(within_budget.record(18'750U, now).has_value());
    // Two seconds over budget, then one second under, repeated.
    const bool under_budget_second = (frame_index / 10) % 3 == 2;
    EXPECT_FALSE(brief_overrun.record(under_budget_second ? 5'000U : 25'000U, now).has_value());
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge::video
