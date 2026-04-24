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
#include <utility>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{
namespace
{

class RecordingPublishedVideoTrack final : public PublishedVideoTrack
{
public:
  RecordingPublishedVideoTrack(std::string name, std::vector<std::string> & unpublished_names)
  : RecordingPublishedVideoTrack(std::move(name), unpublished_names, false)
  {}

  RecordingPublishedVideoTrack(
    std::string name, std::vector<std::string> & unpublished_names, bool simulate_unpublish_failure)
  : PublishedVideoTrack(std::move(name))
  , unpublished_names_(unpublished_names)
  , simulate_unpublish_failure_(simulate_unpublish_failure)
  {}

  ~RecordingPublishedVideoTrack() noexcept override
  {
    try {
      unpublished_names_.push_back(name());
      if (simulate_unpublish_failure_) {
        throw std::runtime_error("simulated video unpublish failure");
      }
    } catch (...) {}
  }

private:
  std::vector<std::string> & unpublished_names_;
  bool simulate_unpublish_failure_ = false;
};

class BestEffortUnpublishRoomConnection final : public RoomConnection
{
public:
  void start(LiveKitConfig, LiveKitRoomDelegate &) override
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

  livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::unique_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    return std::make_unique<RecordingPublishedVideoTrack>(name, unpublished_video_track_names, true);
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

  void start(LiveKitConfig, LiveKitRoomDelegate &) override
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

  livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::unique_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    ++publish_attempts;
    if (publish_attempts == fail_on_attempt_) {
      throw std::runtime_error("simulated video publish failure");
    }

    return std::make_unique<RecordingPublishedVideoTrack>(name, unpublished_video_track_names);
  }

  int publish_attempts = 0;
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

  void start(LiveKitConfig, LiveKitRoomDelegate &) override
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

  livekit::Result<void, livekit::LocalDataTrackTryPushError> tryPushDataTrack(
    const std::shared_ptr<livekit::LocalDataTrack> &, std::vector<std::uint8_t>) override
  {
    return livekit::Result<void, livekit::LocalDataTrackTryPushError>::success();
  }

  void unpublishDataTrack(const std::shared_ptr<livekit::LocalDataTrack> &) override
  {}

  std::unique_ptr<PublishedVideoTrack> publishVideoTrack(
    const std::string & name, const std::shared_ptr<livekit::VideoSource> &, const VideoPublishConfig &) override
  {
    published_video_track_names.push_back(name);
    if (!publish_started_) {
      publish_started_ = true;
      publish_started_promise_.set_value();
    }
    release_publish_future_.wait();

    return std::make_unique<RecordingPublishedVideoTrack>(name, unpublished_video_track_names);
  }

  std::future<void> & publishStarted()
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

std::vector<std::uint8_t> makeFrame(int width, int height)
{
  const auto luma_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const auto chroma_width = static_cast<std::size_t>((width + 1) / 2);
  const auto chroma_height = static_cast<std::size_t>((height + 1) / 2);
  const auto chroma_plane_size = chroma_width * chroma_height;
  return std::vector<std::uint8_t>(luma_size + (2U * chroma_plane_size), 128U);
}

TEST(VideoTrackPublisherTest, RepublishesOnSizeChangeAndUnpublishesOnDestruction)
{
  FakeRoomConnection connection;
  auto publisher =
    std::make_unique<VideoTrackPublisher>(connection, makeSpec("stream:lifecycle", "lkros.video.camera.lifecycle"));

  publisher->write(2, 2, makeFrame(2, 2), 1000);
  publisher->write(2, 2, makeFrame(2, 2), 2000);
  publisher->write(4, 4, makeFrame(4, 4), 3000);
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

TEST(VideoTrackPublisherTest, DestructionUsesBestEffortPublishedTrackCleanup)
{
  BestEffortUnpublishRoomConnection connection;
  auto publisher = std::make_unique<VideoTrackPublisher>(
    connection, makeSpec("stream:unpublish_failure", "lkros.video.camera.unpublish_failure"));

  publisher->write(2, 2, makeFrame(2, 2), 1000);
  EXPECT_NO_THROW(publisher.reset());
  EXPECT_NO_THROW(publisher.reset());

  EXPECT_EQ(connection.published_video_track_names, (std::vector<std::string>{"lkros.video.camera.unpublish_failure"}));
  EXPECT_EQ(
    connection.unpublished_video_track_names, (std::vector<std::string>{"lkros.video.camera.unpublish_failure"}));
}

TEST(VideoTrackPublisherTest, PublishFailureOnFirstFrameCanRetryAndStillDestroyCleanly)
{
  FailNthPublishRoomConnection connection(1);
  auto publisher = std::make_unique<VideoTrackPublisher>(
    connection, makeSpec("stream:publish_retry", "lkros.video.camera.publish_retry"));

  EXPECT_THROW(publisher->write(2, 2, makeFrame(2, 2), 1000), std::runtime_error);
  publisher->write(2, 2, makeFrame(2, 2), 2000);
  publisher.reset();

  EXPECT_EQ(
    connection.published_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.publish_retry",
      "lkros.video.camera.publish_retry",
    }));
  EXPECT_EQ(connection.unpublished_video_track_names, (std::vector<std::string>{"lkros.video.camera.publish_retry"}));
}

TEST(VideoTrackPublisherTest, RepublishFailureLeavesPublisherReadyForRetry)
{
  FailNthPublishRoomConnection connection(2);
  auto publisher = std::make_unique<VideoTrackPublisher>(
    connection, makeSpec("stream:republish_retry", "lkros.video.camera.republish_retry"));

  publisher->write(2, 2, makeFrame(2, 2), 1000);
  EXPECT_THROW(publisher->write(4, 4, makeFrame(4, 4), 2000), std::runtime_error);
  publisher->write(4, 4, makeFrame(4, 4), 3000);
  publisher.reset();

  EXPECT_EQ(
    connection.published_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
    }));
  EXPECT_EQ(
    connection.unpublished_video_track_names,
    (std::vector<std::string>{
      "lkros.video.camera.republish_retry",
      "lkros.video.camera.republish_retry",
    }));
}

TEST(VideoTrackPublisherTest, BlockingPublishStillUnpublishesOnceOnDestruction)
{
  BlockingPublishRoomConnection connection;
  auto publisher = std::make_unique<VideoTrackPublisher>(
    connection, makeSpec("stream:concurrent_shutdown", "lkros.video.camera.concurrent_shutdown"));

  auto write_future = std::async(std::launch::async, [&publisher]() { publisher->write(2, 2, makeFrame(2, 2), 1000); });
  EXPECT_EQ(connection.publishStarted().wait_for(std::chrono::seconds(2)), std::future_status::ready);

  connection.releasePublish();

  EXPECT_EQ(write_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  write_future.get();
  publisher.reset();

  EXPECT_EQ(
    connection.published_video_track_names, (std::vector<std::string>{"lkros.video.camera.concurrent_shutdown"}));
  EXPECT_EQ(
    connection.unpublished_video_track_names, (std::vector<std::string>{"lkros.video.camera.concurrent_shutdown"}));
}

}  // namespace
}  // namespace livekit_ros2_bridge
