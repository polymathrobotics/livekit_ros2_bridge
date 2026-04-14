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
class RecordingObserver final : public VideoStreamLifecycleObserver
{
public:
  void onTrackPublished(int width, int height, bool republished) override
  {
    published_events.emplace_back(width, height, republished);
  }

  void onTrackUnpublish() override
  {
    unpublish_call_count++;
  }

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

  std::vector<std::tuple<int, int, bool>> published_events;
  int unpublish_call_count = 0;
};

class ThrowingUnpublishRoomConnection final : public RoomConnection
{
public:
  void start(LiveKitConfig, RoomEventCallbacks) override
  {}

  void stop() override
  {}

  bool registerRpc(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpc(const std::string &) override
  {
    return true;
  }

  void publishPacket(const OutgoingPacket &) override
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

  std::shared_ptr<VideoTrackHandle> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    auto handle = std::make_shared<VideoTrackHandle>();
    handle->name = name;
    return handle;
  }

  void unpublishVideoTrack(const std::shared_ptr<VideoTrackHandle> & handle) override
  {
    if (handle != nullptr) {
      unpublished_video_track_names.push_back(handle->name);
    }
    throw std::runtime_error("simulated video unpublish failure");
  }

  std::vector<std::string> published_video_track_names;
  std::vector<std::string> unpublished_video_track_names;
};

class FailNthPublishRoomConnection final : public RoomConnection
{
public:
  explicit FailNthPublishRoomConnection(int fail_on_attempt)
  : fail_on_attempt_(fail_on_attempt)
  {}

  void start(LiveKitConfig, RoomEventCallbacks) override
  {}

  void stop() override
  {}

  bool registerRpc(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpc(const std::string &) override
  {
    return true;
  }

  void publishPacket(const OutgoingPacket &) override
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

  std::shared_ptr<VideoTrackHandle> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    ++publish_attempt_count;
    if (publish_attempt_count == fail_on_attempt_) {
      throw std::runtime_error("simulated video publish failure");
    }

    auto handle = std::make_shared<VideoTrackHandle>();
    handle->name = name;
    return handle;
  }

  void unpublishVideoTrack(const std::shared_ptr<VideoTrackHandle> & handle) override
  {
    if (handle != nullptr) {
      unpublished_video_track_names.push_back(handle->name);
    }
  }

  int publish_attempt_count = 0;
  std::vector<std::string> published_video_track_names;
  std::vector<std::string> unpublished_video_track_names;

private:
  int fail_on_attempt_;
};

class BlockingPublishRoomConnection final : public RoomConnection
{
public:
  BlockingPublishRoomConnection()
  : publish_started_future_(publish_started_promise_.get_future())
  , release_publish_future_(release_publish_promise_.get_future().share())
  {}

  void start(LiveKitConfig, RoomEventCallbacks) override
  {}

  void stop() override
  {}

  bool registerRpc(const std::string &, RpcHandler) override
  {
    return true;
  }

  bool unregisterRpc(const std::string &) override
  {
    return true;
  }

  void publishPacket(const OutgoingPacket &) override
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

  std::shared_ptr<VideoTrackHandle> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    if (!publish_started_) {
      publish_started_ = true;
      publish_started_promise_.set_value();
    }
    release_publish_future_.wait();

    auto handle = std::make_shared<VideoTrackHandle>();
    handle->name = name;
    return handle;
  }

  void unpublishVideoTrack(const std::shared_ptr<VideoTrackHandle> & handle) override
  {
    if (handle != nullptr) {
      unpublished_video_track_names.push_back(handle->name);
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

VideoStreamSpec makeVideoStreamSpec(const std::string & stream_key, const std::string & track_name)
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
  FakeRoomConnection connection;
  RecordingObserver observer;
  VideoTrackPublisher publisher(
    connection, makeVideoStreamSpec("stream:lifecycle_observer", "ros.video.camera.lifecycle_observer"), observer);

  publisher.write(2, 2, makeI420Frame(2, 2), 1000);
  publisher.write(2, 2, makeI420Frame(2, 2), 2000);
  publisher.write(4, 4, makeI420Frame(4, 4), 3000);
  publisher.shutdown();
  publisher.write(8, 8, makeI420Frame(8, 8), 4000);
  publisher.shutdown();

  EXPECT_EQ(
    observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
      std::make_tuple(4, 4, true),
    }));
  EXPECT_EQ(observer.unpublish_call_count, 1);
  EXPECT_EQ(
    connection.state->event_log,
    (std::vector<std::string>{
      "publish_video_track:ros.video.camera.lifecycle_observer",
      "unpublish_video_track:ros.video.camera.lifecycle_observer",
      "publish_video_track:ros.video.camera.lifecycle_observer",
      "unpublish_video_track:ros.video.camera.lifecycle_observer",
    }));
}

TEST(VideoTrackPublisherTest, ShutdownSwallowsVideoUnpublishFailureAndStaysClosed)
{
  ThrowingUnpublishRoomConnection connection;
  RecordingObserver observer;
  VideoTrackPublisher publisher(
    connection, makeVideoStreamSpec("stream:unpublish_failure", "ros.video.camera.unpublish_failure"), observer);

  publisher.write(2, 2, makeI420Frame(2, 2), 1000);
  EXPECT_NO_THROW(publisher.shutdown());
  publisher.write(4, 4, makeI420Frame(4, 4), 2000);
  EXPECT_NO_THROW(publisher.shutdown());

  EXPECT_EQ(connection.published_video_track_names, (std::vector<std::string>{"ros.video.camera.unpublish_failure"}));
  EXPECT_EQ(connection.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.unpublish_failure"}));
  EXPECT_EQ(observer.unpublish_call_count, 1);
}

TEST(VideoTrackPublisherTest, PublishFailureOnFirstFrameCanRetryAndStillShutdownCleanly)
{
  FailNthPublishRoomConnection connection(1);
  RecordingObserver observer;
  VideoTrackPublisher publisher(
    connection, makeVideoStreamSpec("stream:publish_retry", "ros.video.camera.publish_retry"), observer);

  EXPECT_THROW(publisher.write(2, 2, makeI420Frame(2, 2), 1000), std::runtime_error);
  publisher.write(2, 2, makeI420Frame(2, 2), 2000);
  publisher.shutdown();

  EXPECT_EQ(
    connection.published_video_track_names,
    (std::vector<std::string>{
      "ros.video.camera.publish_retry",
      "ros.video.camera.publish_retry",
    }));
  EXPECT_EQ(connection.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.publish_retry"}));
  EXPECT_EQ(
    observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
    }));
  EXPECT_EQ(observer.unpublish_call_count, 1);
}

TEST(VideoTrackPublisherTest, RepublishFailureLeavesPublisherReadyForRetryWithoutDoubleUnpublish)
{
  FailNthPublishRoomConnection connection(2);
  RecordingObserver observer;
  VideoTrackPublisher publisher(
    connection, makeVideoStreamSpec("stream:republish_retry", "ros.video.camera.republish_retry"), observer);

  publisher.write(2, 2, makeI420Frame(2, 2), 1000);
  EXPECT_THROW(publisher.write(4, 4, makeI420Frame(4, 4), 2000), std::runtime_error);
  publisher.write(4, 4, makeI420Frame(4, 4), 3000);
  publisher.shutdown();

  EXPECT_EQ(
    connection.published_video_track_names,
    (std::vector<std::string>{
      "ros.video.camera.republish_retry",
      "ros.video.camera.republish_retry",
      "ros.video.camera.republish_retry",
    }));
  EXPECT_EQ(
    connection.unpublished_video_track_names,
    (std::vector<std::string>{
      "ros.video.camera.republish_retry",
      "ros.video.camera.republish_retry",
    }));
  EXPECT_EQ(
    observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
      std::make_tuple(4, 4, true),
    }));
  EXPECT_EQ(observer.unpublish_call_count, 1);
}

TEST(VideoTrackPublisherTest, ShutdownWaitsForInFlightPublishThenUnpublishesOnce)
{
  BlockingPublishRoomConnection connection;
  RecordingObserver observer;
  VideoTrackPublisher publisher(
    connection, makeVideoStreamSpec("stream:concurrent_shutdown", "ros.video.camera.concurrent_shutdown"), observer);

  auto write_future =
    std::async(std::launch::async, [&publisher]() { publisher.write(2, 2, makeI420Frame(2, 2), 1000); });
  EXPECT_EQ(connection.publishStartedFuture().wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto shutdown_future = std::async(std::launch::async, [&publisher]() { publisher.shutdown(); });
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  connection.releasePublish();

  EXPECT_EQ(write_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  write_future.get();
  EXPECT_EQ(shutdown_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  shutdown_future.get();

  EXPECT_EQ(
    observer.published_events,
    (std::vector<std::tuple<int, int, bool>>{
      std::make_tuple(2, 2, false),
    }));
  EXPECT_EQ(observer.unpublish_call_count, 1);
  EXPECT_EQ(connection.published_video_track_names, (std::vector<std::string>{"ros.video.camera.concurrent_shutdown"}));
  EXPECT_EQ(
    connection.unpublished_video_track_names, (std::vector<std::string>{"ros.video.camera.concurrent_shutdown"}));
}

}  // namespace
}  // namespace livekit_ros2_bridge
