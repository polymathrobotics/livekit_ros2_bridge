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
#include <tuple>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{
namespace
{
class RecordingVideoStreamLifecycleObserver final : public VideoStreamLifecycleObserver
{
public:
  void onVideoTrackPublished(int width, int height, bool republished) override
  {
    published_events.emplace_back(width, height, republished);
  }

  void onVideoTrackUnpublishing() override
  {
    unpublish_call_count++;
  }

  void onVideoStreamSampleUnpackFailed(const std::string &) override
  {}

  void onVideoStreamCaptureFailed(const std::string &) override
  {}

  void onVideoStreamPipelineFailed(const std::string &) override
  {}

  void onVideoStreamRestartFailed(const std::string &) override
  {}

  void onVideoStreamPushFailed(const std::string &) override
  {}

  std::vector<std::tuple<int, int, bool>> published_events;
  int unpublish_call_count = 0;
};

class ThrowingVideoUnpublishRoomConnection final : public RoomConnection
{
public:
  void start(
    RoomConnectionConfig,
    std::string,
    RoomConnectionCallbacks,
    std::chrono::milliseconds,
    std::chrono::milliseconds) override
  {}

  void stop() override
  {}

  bool registerRpcMethod(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpcMethod(const std::string &) override
  {
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket &) override
  {}

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string &) override
  {
    return nullptr;
  }

  DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return DataTrackPushResult::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(track_name);
    auto track = std::make_shared<PublishedVideoTrack>();
    track->track_name = track_name;
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) override
  {
    if (track != nullptr) {
      unpublished_video_track_names.push_back(track->track_name);
    }
    throw std::runtime_error("simulated video unpublish failure");
  }

  std::vector<std::string> published_video_track_names;
  std::vector<std::string> unpublished_video_track_names;
};

class FailOnceVideoPublishRoomConnection final : public RoomConnection
{
public:
  void start(
    RoomConnectionConfig,
    std::string,
    RoomConnectionCallbacks,
    std::chrono::milliseconds,
    std::chrono::milliseconds) override
  {}

  void stop() override
  {}

  bool registerRpcMethod(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpcMethod(const std::string &) override
  {
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket &) override
  {}

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string &) override
  {
    return nullptr;
  }

  DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return DataTrackPushResult::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(track_name);
    ++publish_attempt_count;
    if (publish_attempt_count == 1) {
      throw std::runtime_error("simulated video publish failure");
    }

    auto track = std::make_shared<PublishedVideoTrack>();
    track->track_name = track_name;
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) override
  {
    if (track != nullptr) {
      unpublished_video_track_names.push_back(track->track_name);
    }
  }

  int publish_attempt_count = 0;
  std::vector<std::string> published_video_track_names;
  std::vector<std::string> unpublished_video_track_names;
};

class BlockingVideoPublishRoomConnection final : public RoomConnection
{
public:
  BlockingVideoPublishRoomConnection()
  : publish_started_future_(publish_started_promise_.get_future())
  , release_publish_future_(release_publish_promise_.get_future().share())
  {}

  void start(
    RoomConnectionConfig,
    std::string,
    RoomConnectionCallbacks,
    std::chrono::milliseconds,
    std::chrono::milliseconds) override
  {}

  void stop() override
  {}

  bool registerRpcMethod(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpcMethod(const std::string &) override
  {
    return true;
  }

  void publishControlPacket(const OutgoingControlPacket &) override
  {}

  std::shared_ptr<livekit::LocalDataTrack> publishDataTrack(const std::string &) override
  {
    return nullptr;
  }

  DataTrackPushResult tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return DataTrackPushResult::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::shared_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & track_name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(track_name);
    if (!publish_started_) {
      publish_started_ = true;
      publish_started_promise_.set_value();
    }
    release_publish_future_.wait();

    auto track = std::make_shared<PublishedVideoTrack>();
    track->track_name = track_name;
    return track;
  }

  void unpublishVideoTrack(const std::shared_ptr<PublishedVideoTrack> & track) override
  {
    if (track != nullptr) {
      unpublished_video_track_names.push_back(track->track_name);
    }
  }

  std::future<void> & publishStartedFuture()
  {
    return publish_started_future_;
  }

  void releasePublish()
  {
    release_publish_promise_.set_value();
  }

  std::vector<std::string> published_video_track_names;
  std::vector<std::string> unpublished_video_track_names;

private:
  bool publish_started_ = false;
  std::promise<void> publish_started_promise_;
  std::future<void> publish_started_future_;
  std::promise<void> release_publish_promise_;
  std::shared_future<void> release_publish_future_;
};

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

TEST(VideoTrackPublisherTest, LifecycleObserverTracksRepublishAndIgnoresFramesAfterShutdown)
{
  FakeRoomConnection session;
  RecordingVideoStreamLifecycleObserver lifecycle_observer;
  VideoTrackPublisher publisher(
    session, makeSpec("stream:lifecycle_observer", "ros.video.camera.lifecycle_observer"), lifecycle_observer);

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);
  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 2000);
  publisher.handleFrame(4, 4, makeI420Frame(4, 4), 3000);
  publisher.shutdown();
  publisher.handleFrame(8, 8, makeI420Frame(8, 8), 4000);
  publisher.shutdown();

  EXPECT_EQ(
    lifecycle_observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
      std::make_tuple(4, 4, true),
    }));
  EXPECT_EQ(lifecycle_observer.unpublish_call_count, 1);

  const auto expected_event_log = std::vector<std::string>{
    "publish_video_track:ros.video.camera.lifecycle_observer",
    "unpublish_video_track:ros.video.camera.lifecycle_observer",
    "publish_video_track:ros.video.camera.lifecycle_observer",
    "unpublish_video_track:ros.video.camera.lifecycle_observer",
  };

  EXPECT_EQ(session.state->event_log, expected_event_log);
}

TEST(VideoTrackPublisherTest, ShutdownSwallowsVideoUnpublishFailureAndStaysClosed)
{
  ThrowingVideoUnpublishRoomConnection session;
  RecordingVideoStreamLifecycleObserver lifecycle_observer;
  VideoTrackPublisher publisher(
    session, makeSpec("stream:unpublish_failure", "ros.video.camera.unpublish_failure"), lifecycle_observer);

  publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000);
  EXPECT_NO_THROW(publisher.shutdown());
  publisher.handleFrame(4, 4, makeI420Frame(4, 4), 2000);
  EXPECT_NO_THROW(publisher.shutdown());

  EXPECT_EQ(session.published_video_track_names, (std::vector<std::string>{"ros.video.camera.unpublish_failure"}));
  EXPECT_EQ(session.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.unpublish_failure"}));
  EXPECT_EQ(lifecycle_observer.unpublish_call_count, 1);
}

TEST(VideoTrackPublisherTest, PublishFailureOnFirstFrameCanRetryAndStillShutdownCleanly)
{
  FailOnceVideoPublishRoomConnection session;
  RecordingVideoStreamLifecycleObserver lifecycle_observer;
  VideoTrackPublisher publisher(
    session, makeSpec("stream:publish_retry", "ros.video.camera.publish_retry"), lifecycle_observer);

  EXPECT_THROW(publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000), std::runtime_error);
  EXPECT_NO_THROW(publisher.handleFrame(2, 2, makeI420Frame(2, 2), 2000));
  EXPECT_NO_THROW(publisher.shutdown());

  EXPECT_EQ(
    session.published_video_track_names,
    (std::vector<std::string>{
      "ros.video.camera.publish_retry",
      "ros.video.camera.publish_retry",
    }));
  EXPECT_EQ(session.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.publish_retry"}));
  EXPECT_EQ(
    lifecycle_observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
    }));
  EXPECT_EQ(lifecycle_observer.unpublish_call_count, 1);
}

TEST(VideoTrackPublisherTest, ShutdownWaitsForInFlightPublishThenUnpublishesOnce)
{
  BlockingVideoPublishRoomConnection session;
  RecordingVideoStreamLifecycleObserver lifecycle_observer;
  VideoTrackPublisher publisher(
    session, makeSpec("stream:concurrent_shutdown", "ros.video.camera.concurrent_shutdown"), lifecycle_observer);

  auto handle_frame_future =
    std::async(std::launch::async, [&publisher]() { publisher.handleFrame(2, 2, makeI420Frame(2, 2), 1000); });
  EXPECT_EQ(session.publishStartedFuture().wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto shutdown_future = std::async(std::launch::async, [&publisher]() { publisher.shutdown(); });
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  session.releasePublish();

  EXPECT_EQ(handle_frame_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NO_THROW(handle_frame_future.get());
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NO_THROW(shutdown_future.get());

  EXPECT_EQ(
    lifecycle_observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
    }));
  EXPECT_EQ(lifecycle_observer.unpublish_call_count, 1);
  EXPECT_EQ(session.published_video_track_names, (std::vector<std::string>{"ros.video.camera.concurrent_shutdown"}));
  EXPECT_EQ(session.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.concurrent_shutdown"}));
}

}  // namespace
}  // namespace livekit_ros2_bridge
