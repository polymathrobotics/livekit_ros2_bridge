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

#include "video_stream_manager.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "livekit/video_frame.h"
#include "livekit/video_source.h"
#include "rclcpp/logging.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "video_ingestor.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kVideoStreamManagerLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_manager");

}  // namespace

class VideoStreamManager::StreamRecord final : public IVideoFrameSink
{
public:
  StreamRecord(
    rclcpp::Node & node,
    RoomSession & session,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config)
  : node_(node)
  , session_(session)
  , spec_(std::move(spec))
  , subscription_qos_config_(subscription_qos_config)
  {}

  ~StreamRecord()
  {
    shutdown();
  }

  std::string ensureRunning()
  {
    std::shared_ptr<IVideoIngestor> ingestor;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        throw std::runtime_error("Video stream is shut down.");
      }

      if (!ingestor_) {
        ingestor_ = createIngestorLocked();
      }
      ingestor = ingestor_;
    }

    ingestor->ensureRunning();
    return spec_.track_name;
  }

  void shutdown()
  {
    std::shared_ptr<IVideoIngestor> ingestor;
    std::shared_ptr<PublishedVideoTrack> published_track;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return;
      }

      is_shutdown_ = true;
      ingestor = std::move(ingestor_);
      published_track = std::move(published_track_);
      video_source_.reset();
      published_width_ = 0;
      published_height_ = 0;
    }

    if (ingestor) {
      ingestor->shutdown();
    }
    if (published_track) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_track_unpublishing")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .info();
      session_.unpublishVideoTrack(published_track);
    }
  }

  void handleFrame(int width, int height, std::vector<std::uint8_t> i420, std::int64_t timestamp_us) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }

    ensurePublishedTrackLocked(width, height);
    // The public LiveKit C++ SDK takes an owned VideoFrame buffer here, so this
    // remains a low-copy path rather than true zero-copy. Keeping the data in
    // I420 still avoids the more expensive per-frame color conversion.
    livekit::VideoFrame frame(width, height, livekit::VideoBufferType::I420, std::move(i420));
    video_source_->captureFrame(frame, timestamp_us);
  }

private:
  std::shared_ptr<IVideoIngestor> createIngestorLocked()
  {
    if (spec_.source_kind == VideoSourceKind::ConfiguredSource) {
      return makeConfiguredSourceVideoIngestor(spec_, *this);
    }
    if (spec_.source_kind == VideoSourceKind::RosTopic && spec_.ingest_mode == kRawImageIngestMode) {
      return makeRawRosVideoIngestor(node_, spec_, subscription_qos_config_, *this);
    }
    if (spec_.source_kind == VideoSourceKind::RosTopic && spec_.ingest_mode == kCompressedImageIngestMode) {
      return makeCompressedRosVideoIngestor(node_, spec_, subscription_qos_config_, *this);
    }

    throw std::runtime_error(
      "Unsupported video source kind/ingest mode combination '" + videoSourceKindToString(spec_.source_kind) + "/" +
      spec_.ingest_mode + "'.");
  }

  void ensurePublishedTrackLocked(int width, int height)
  {
    if (
      video_source_ != nullptr && published_track_ != nullptr && published_width_ == width &&
      published_height_ == height)
    {
      return;
    }

    const bool republishing = published_track_ != nullptr;
    if (published_track_) {
      LogEvent(kVideoStreamManagerLogger, "video_stream_track_replacing")
        .field("stream_key", spec_.stream_key)
        .field("track_name", spec_.track_name)
        .field("previous_width", published_width_)
        .field("previous_height", published_height_)
        .field("next_width", width)
        .field("next_height", height)
        .info();
      session_.unpublishVideoTrack(published_track_);
      published_track_.reset();
    }

    video_source_ = std::make_shared<livekit::VideoSource>(width, height);
    published_track_ = session_.publishVideoTrack(spec_.track_name, video_source_, spec_.publish_config);
    published_width_ = width;
    published_height_ = height;

    LogEvent(
      kVideoStreamManagerLogger, republishing ? "video_stream_track_republished" : "video_stream_track_published")
      .field("stream_key", spec_.stream_key)
      .field("track_name", spec_.track_name)
      .field("width", width)
      .field("height", height)
      .info();
  }

  rclcpp::Node & node_;
  RoomSession & session_;
  VideoStreamSpec spec_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  std::shared_ptr<IVideoIngestor> ingestor_;
  std::shared_ptr<livekit::VideoSource> video_source_;
  std::shared_ptr<PublishedVideoTrack> published_track_;
  int published_width_ = 0;
  int published_height_ = 0;
  const SubscriptionQosConfig * subscription_qos_config_;
};

VideoStreamManager::VideoStreamManager(
  rclcpp::Node & node, RoomSession & session, const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, session_(session)
, subscription_qos_config_(subscription_qos_config)
{}

VideoStreamManager::~VideoStreamManager()
{
  shutdown();
}

std::string VideoStreamManager::ensureStream(const VideoStreamSpec & spec)
{
  std::shared_ptr<StreamRecord> record;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream manager is shut down.");
    }

    auto [it, inserted] = streams_.try_emplace(spec.stream_key);
    if (inserted) {
      it->second = std::make_shared<StreamRecord>(node_, session_, spec, subscription_qos_config_);
    }
    record = it->second;
  }

  return record->ensureRunning();
}

void VideoStreamManager::stopStream(const std::string & stream_key)
{
  std::shared_ptr<StreamRecord> record;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = streams_.find(stream_key);
    if (it == streams_.end()) {
      return;
    }
    record = std::move(it->second);
    streams_.erase(it);
  }

  if (record) {
    record->shutdown();
  }
}

void VideoStreamManager::shutdown()
{
  std::unordered_map<std::string, std::shared_ptr<StreamRecord>> streams;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    is_shutdown_ = true;
    streams = std::move(streams_);
    streams_.clear();
  }

  for (auto & entry : streams) {
    if (entry.second) {
      entry.second->shutdown();
    }
  }
}

}  // namespace livekit_ros2_bridge
