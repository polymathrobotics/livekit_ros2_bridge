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
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "livekit/video_frame.h"
#include "video/track_publisher.hpp"

namespace livekit_ros2_bridge::video
{
namespace
{

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

TEST(TrackPublisherTest, EnablesUserTimestampPacketTrailerForPublishedVideo)
{
  FakeRoomConnection connection;
  auto publisher =
    std::make_unique<TrackPublisher>(connection, makeSpec("stream:metadata", "lkros.video.camera.metadata"));

  publisher->capture(makeFrame(2, 2), 1000);

  ASSERT_EQ(connection.state->published_video_options.size(), 1U);
  EXPECT_TRUE(connection.state->published_video_options.front().packet_trailer_features.user_timestamp);
  EXPECT_FALSE(connection.state->published_video_options.front().packet_trailer_features.frame_id);
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

}  // namespace
}  // namespace livekit_ros2_bridge::video
