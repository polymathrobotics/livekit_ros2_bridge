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
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "livekit/encoded_video_source.h"
#include "livekit/video_frame.h"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "support/scoped_log_capture.hpp"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{
namespace
{

// LiveKit 1.6.0+ requires livekit::initialize() before constructing livekit::VideoSource; see ScopedLiveKitInit.
const test_support::ScopedLiveKitInit kLiveKitInit;

// Same as TrackPublisher's private intervals, plus extra time to be safe.
constexpr auto kKeyFrameRequestInterval = std::chrono::milliseconds(500);
constexpr auto kLogThrottle = std::chrono::seconds(5);
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
  spec.publish_options.video_codec = livekit::VideoCodec::H264;
  spec.publish_options.video_encoder = livekit::VideoEncoderBackend::PreEncoded;
  spec.publish_options.simulcast = false;
  return spec;
}

livekit::EncodedVideoSource::Frame makeEncodedFrame(bool is_keyframe, std::uint32_t width, std::uint32_t height)
{
  livekit::EncodedVideoSource::Frame frame;
  frame.is_keyframe = is_keyframe;
  frame.width = width;
  frame.height = height;
  frame.timestamp_us = 1000;
  // Annex-B start code and an IDR (type 5) or non-IDR (type 1) slice header.
  const std::uint8_t nal_header = is_keyframe ? 0x65U : 0x41U;
  frame.data = {0x00U, 0x00U, 0x00U, 0x01U, nal_header, 0x88U};
  return frame;
}

// LiveKit rejects an empty frame, so sending this one fails.
livekit::EncodedVideoSource::Frame makeEmptyDeltaFrame()
{
  livekit::EncodedVideoSource::Frame frame = makeEncodedFrame(false, 64, 48);
  frame.data.clear();
  return frame;
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
  // Two discarded frames, one request: throttled.
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
  ASSERT_EQ(connection.state->published_video_sources.size(), 1U);
  EXPECT_NE(
    std::dynamic_pointer_cast<livekit::EncodedVideoSource>(connection.state->published_video_sources.front()), nullptr);
  // Throttled: one line, logged at the first discarded frame.
  EXPECT_EQ(
    logs.fieldValues(
      {"video_stream_passthrough_waiting_for_key_frame", "stream_key=stream:passthrough"}, "discarded_frames"),
    std::vector<std::string>{"1"});
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_waiting_for_key_frame", "stream_key=stream:passthrough"}, "count"),
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
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_viewer_key_frame_request"}), 0U);
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.restart"}));
  EXPECT_EQ(connection.state->unpublished_video_track_names, (std::vector<std::string>{"lkros.video.restart"}));
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:restart"}, "discarded_frames"),
    std::vector<std::string>{"2"});
  // Only the first wait logs: later ones follow a logged restart or discontinuity.
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_waiting_for_key_frame"}), 0U);
}

TEST(TrackPublisherTest, PassthroughDiscontinuousDeltaFrameWaitsForNextKeyFrame)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:gap", "lkros.video.gap"));

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  EXPECT_EQ(key_frame_requests, 1);
  // Still waiting: no new request, and another gap changes nothing.
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));

  // The next gap asks again right away; its warning is throttled.
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  EXPECT_EQ(key_frame_requests, 2);
  publisher.reset();

  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:gap"}, "discarded_frames"),
    std::vector<std::string>{"3"});
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_discontinuity", "stream_key=stream:gap"}, "count"),
    std::vector<std::string>{"1"});
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.gap"}));
}

// The first buffer of every run is marked discontinuous, but the publisher is already waiting for a keyframe.
TEST(TrackPublisherTest, PassthroughStartOfRunIsNotLoggedAsAGap)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(connection, makePassthroughSpec("stream:runs", "lkros.video.runs"));

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame, true));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  publisher->awaitKeyFrame();
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_discontinuity"}), 0U);
}

// Publishing stalls the sample thread, so frames may be dropped meanwhile; that gap is not warned about.
TEST(TrackPublisherTest, PassthroughGapRightAfterPublishWaitsForKeyFrameWithoutWarning)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makePassthroughSpec("stream:slow_publish", "lkros.video.slow_publish"));

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_discontinuity"}), 0U);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  // Only the frame right after the publish is exempt.
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame, true));
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 2);
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_discontinuity", "stream_key=stream:slow_publish"}), 1U);
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_resumed", "stream_key=stream:slow_publish"}, "discarded_frames"),
    std::vector<std::string>{"1"});
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

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  bool sent = true;
  EXPECT_NO_THROW(sent = publisher->captureEncoded(makeEmptyDeltaFrame(), request_key_frame));
  EXPECT_FALSE(sent);
  // The picture is broken until the next keyframe.
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 1);

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  publisher.reset();

  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.send"}));
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_send_failed", "stream_key=stream:send"}, "count"),
    std::vector<std::string>{"1"});
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
  for (int failed_send = 0; failed_send < kFailedSends; ++failed_send) {
    publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame);
    publisher->captureEncoded(makeEmptyDeltaFrame(), request_key_frame);
    publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame);
  }
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  publisher.reset();

  EXPECT_GE(key_frame_requests, 1);
  EXPECT_LE(key_frame_requests, 1 + elapsed / kKeyFrameRequestInterval);
  const std::vector<std::string> counts =
    logs.fieldValues({"video_stream_passthrough_send_failed", "stream_key=stream:send_storm"}, "count");
  ASSERT_FALSE(counts.empty());
  EXPECT_EQ(counts.front(), "1");
  EXPECT_LE(counts.size(), 1U + static_cast<std::size_t>(elapsed / kLogThrottle));
  EXPECT_EQ(connection.state->published_video_track_names, (std::vector<std::string>{"lkros.video.send_storm"}));
}

// LiveKit asks for a keyframe once per gap and drops delta frames until it gets one, so a request
// the throttle holds back must be made later, not lost.
TEST(TrackPublisherTest, PassthroughLiveKitKeyFrameRequestWaitsOutTheThrottle)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  bool livekit_wants_key_frame = false;
  const TrackPublisher::EncodedFeedbackReader read_feedback = [&](const livekit::EncodedVideoSource &) {
    livekit::EncodedVideoSource::Feedback feedback;
    feedback.keyframe_requested = std::exchange(livekit_wants_key_frame, false);
    return feedback;
  };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makePassthroughSpec("stream:viewer", "lkros.video.viewer"), read_feedback);

  // The discarded delta frame asks for a keyframe, which starts the throttle.
  EXPECT_FALSE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  livekit_wants_key_frame = true;
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_EQ(logs.countLines({"video_stream_passthrough_viewer_key_frame_request"}), 0U);

  std::this_thread::sleep_for(kKeyFrameRequestInterval + kTimingMargin);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 2);
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_viewer_key_frame_request", "stream_key=stream:viewer"}, "count"),
    std::vector<std::string>{"1"});

  // A sent keyframe answers the request.
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  std::this_thread::sleep_for(kKeyFrameRequestInterval + kTimingMargin);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 2);
}

TEST(TrackPublisherTest, PassthroughFailedFeedbackReadKeepsSendingAndKeepsTheKeyFrameRequest)
{
  test_support::ScopedLogCapture logs;
  FakeRoomConnection connection;
  int key_frame_requests = 0;
  const std::function<void()> request_key_frame = [&]() { ++key_frame_requests; };
  int feedback_reads = 0;
  const TrackPublisher::EncodedFeedbackReader read_feedback = [&](const livekit::EncodedVideoSource &) {
    ++feedback_reads;
    livekit::EncodedVideoSource::Feedback feedback;
    // The first read arrives with the keyframe that was just sent, so the request outlives it.
    feedback.keyframe_requested = feedback_reads == 1;
    if (feedback_reads == 2 || feedback_reads == 3) {
      throw std::runtime_error("feedback unavailable");
    }
    return feedback;
  };
  auto publisher = std::make_unique<TrackPublisher>(
    connection, makePassthroughSpec("stream:feedback", "lkros.video.feedback"), read_feedback);

  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(true, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 1);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 1);

  std::this_thread::sleep_for(kKeyFrameRequestInterval + kTimingMargin);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 2);

  // An empty read doesn't answer the request either.
  std::this_thread::sleep_for(kKeyFrameRequestInterval + kTimingMargin);
  EXPECT_TRUE(publisher->captureEncoded(makeEncodedFrame(false, 64, 48), request_key_frame));
  EXPECT_EQ(key_frame_requests, 3);

  // The second failure falls inside the log throttle.
  EXPECT_EQ(
    logs.fieldValues({"video_stream_passthrough_feedback_failed", "stream_key=stream:feedback"}, "count"),
    std::vector<std::string>{"1"});
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

TEST(PassthroughBitrateMonitorTest, StaysQuietWithoutATarget)
{
  using std::chrono::milliseconds;
  PassthroughBitrateMonitor monitor;
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(100);

  for (int frame_index = 0; frame_index <= 100; ++frame_index) {
    EXPECT_FALSE(monitor.record(25'000U, start + milliseconds(100 * frame_index)).has_value());
  }
}

TEST(PassthroughBitrateMonitorTest, StaysQuietAtExactlyOnePointFiveTimesTheTarget)
{
  using std::chrono::milliseconds;
  PassthroughBitrateMonitor monitor;
  monitor.setTarget(1'000'000U);
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(100);

  // 18.75 kB every 100 ms is 1.5 Mbit/s, exactly 1.5x the target, which is not over budget.
  for (int frame_index = 0; frame_index <= 100; ++frame_index) {
    EXPECT_FALSE(monitor.record(18'750U, start + milliseconds(100 * frame_index)).has_value());
  }
}

TEST(PassthroughBitrateMonitorTest, StaysQuietWhenTheOverrunIsBrief)
{
  using std::chrono::milliseconds;
  PassthroughBitrateMonitor monitor;
  monitor.setTarget(1'000'000U);
  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(100);

  // Two seconds over budget, then one second under, repeated.
  for (int frame_index = 0; frame_index <= 100; ++frame_index) {
    const bool under_budget_second = (frame_index / 10) % 3 == 2;
    const auto reported =
      monitor.record(under_budget_second ? 5'000U : 25'000U, start + milliseconds(100 * frame_index));
    EXPECT_FALSE(reported.has_value());
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge::video
