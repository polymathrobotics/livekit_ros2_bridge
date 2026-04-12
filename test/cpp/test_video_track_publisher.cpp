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
#include <string>
#include <vector>

#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{
namespace
{
using test_support::ScopedRclcppInit;

VideoStreamSpec makeSpec(const std::string & stream_key, const std::string & track_name)
{
  VideoStreamSpec spec;
  spec.stream_key = stream_key;
  spec.track_name = track_name;
  return spec;
}

std::vector<std::uint8_t> makeI420Frame(int width, int height)
{
  const auto luma_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const auto chroma_width = static_cast<std::size_t>((width + 1) / 2);
  const auto chroma_height = static_cast<std::size_t>((height + 1) / 2);
  const auto chroma_plane_size = chroma_width * chroma_height;
  return std::vector<std::uint8_t>(luma_size + (2U * chroma_plane_size), 128U);
}

TEST(VideoTrackPublisherTest, PublishesOnFirstFrame)
{
  ScopedRclcppInit init;
  FakeRoomSession session;
  VideoTrackPublisher publisher(session, makeSpec("stream:first_frame", "ros.video.camera.first_frame"));

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], "ros.video.camera.first_frame");
  EXPECT_TRUE(session.state->unpublished_video_track_names.empty());
}

TEST(VideoTrackPublisherTest, SameSizeFramesReuseCurrentPublication)
{
  ScopedRclcppInit init;
  FakeRoomSession session;
  VideoTrackPublisher publisher(session, makeSpec("stream:same_size", "ros.video.camera.same_size"));

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);
  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 2000);

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], "ros.video.camera.same_size");
  EXPECT_TRUE(session.state->unpublished_video_track_names.empty());
}

TEST(VideoTrackPublisherTest, DimensionChangeRepublishesTrack)
{
  ScopedRclcppInit init;
  FakeRoomSession session;
  VideoTrackPublisher publisher(session, makeSpec("stream:resize", "ros.video.camera.resize"));

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);
  publisher.handleFrame(4, 4, makeI420Frame(4, 4), 2000);

  ASSERT_EQ(session.state->published_video_track_names.size(), 2U);
  EXPECT_EQ(session.state->published_video_track_names[0], "ros.video.camera.resize");
  EXPECT_EQ(session.state->published_video_track_names[1], "ros.video.camera.resize");
  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], "ros.video.camera.resize");
  ASSERT_EQ(session.state->event_log.size(), 3U);
  EXPECT_EQ(session.state->event_log[0], "publish_video_track:ros.video.camera.resize");
  EXPECT_EQ(session.state->event_log[1], "unpublish_video_track:ros.video.camera.resize");
  EXPECT_EQ(session.state->event_log[2], "publish_video_track:ros.video.camera.resize");
}

TEST(VideoTrackPublisherTest, ShutdownUnpublishesActiveTrack)
{
  ScopedRclcppInit init;
  FakeRoomSession session;
  VideoTrackPublisher publisher(session, makeSpec("stream:shutdown", "ros.video.camera.shutdown"));

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);
  publisher.shutdown();

  ASSERT_EQ(session.state->published_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->published_video_track_names[0], "ros.video.camera.shutdown");
  ASSERT_EQ(session.state->unpublished_video_track_names.size(), 1U);
  EXPECT_EQ(session.state->unpublished_video_track_names[0], "ros.video.camera.shutdown");

  publisher.shutdown();
  EXPECT_EQ(session.state->unpublished_video_track_names.size(), 1U);
}

}  // namespace
}  // namespace livekit_ros2_bridge
