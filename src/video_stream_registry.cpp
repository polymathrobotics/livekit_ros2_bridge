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

#include "video_stream_registry.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "subscription_qos.hpp"
#include "video_frame_source.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

// Each instance owns the source side and publication side for one resolved
// stream: one VideoFrameSource feeds one VideoTrackPublisher.
class VideoStreamRegistry::VideoStreamInstance final
{
public:
  VideoStreamInstance(
    rclcpp::Node & node,
    RoomSession & session,
    VideoStreamSpec spec,
    const SubscriptionQosConfig * subscription_qos_config)
  : node_(node)
  , spec_(std::move(spec))
  , subscription_qos_config_(subscription_qos_config)
  , video_track_publisher_(std::make_unique<VideoTrackPublisher>(session, spec_))
  {}

  ~VideoStreamInstance()
  {
    shutdown();
  }

  std::string ensureRunning()
  {
    std::shared_ptr<VideoFrameSource> frame_source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        throw std::runtime_error("Video stream is shut down.");
      }

      if (!frame_source_) {
        frame_source_ = createFrameSourceLocked();
      }
      frame_source = frame_source_;
    }

    frame_source->ensureRunning();
    return spec_.track_name;
  }

  void shutdown()
  {
    std::shared_ptr<VideoFrameSource> frame_source;
    std::unique_ptr<VideoTrackPublisher> video_track_publisher;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_shutdown_) {
        return;
      }

      is_shutdown_ = true;
      frame_source = std::move(frame_source_);
      video_track_publisher = std::move(video_track_publisher_);
    }

    if (frame_source) {
      frame_source->shutdown();
    }
    if (video_track_publisher) {
      video_track_publisher->shutdown();
    }
  }

private:
  std::shared_ptr<VideoFrameSource> createFrameSourceLocked()
  {
    if (spec_.input_kind == VideoInputKind::ConfiguredSource) {
      return makeConfiguredSourceVideoFrameSource(spec_, *video_track_publisher_);
    }
    if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kRawImageIngestMode) {
      return makeRawRosVideoFrameSource(node_, spec_, subscription_qos_config_, *video_track_publisher_);
    }
    if (spec_.input_kind == VideoInputKind::RosTopic && spec_.ingest_mode == kCompressedImageIngestMode) {
      return makeCompressedRosVideoFrameSource(node_, spec_, subscription_qos_config_, *video_track_publisher_);
    }

    throw std::runtime_error(
      "Unsupported video input kind/ingest mode combination '" + videoInputKindToString(spec_.input_kind) + "/" +
      spec_.ingest_mode + "'.");
  }

  rclcpp::Node & node_;
  VideoStreamSpec spec_;
  const SubscriptionQosConfig * subscription_qos_config_;
  std::mutex mutex_;
  bool is_shutdown_ = false;
  std::shared_ptr<VideoFrameSource> frame_source_;
  std::unique_ptr<VideoTrackPublisher> video_track_publisher_;
};

VideoStreamRegistry::VideoStreamRegistry(
  rclcpp::Node & node, RoomSession & session, const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, session_(session)
, subscription_qos_config_(subscription_qos_config)
{}

VideoStreamRegistry::~VideoStreamRegistry()
{
  shutdown();
}

std::string VideoStreamRegistry::ensureStreamRunning(const VideoStreamSpec & spec)
{
  std::shared_ptr<VideoStreamInstance> instance;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      throw std::runtime_error("Video stream registry is shut down.");
    }

    auto [it, inserted] = stream_instances_.try_emplace(spec.stream_key);
    if (inserted) {
      it->second = std::make_shared<VideoStreamInstance>(node_, session_, spec, subscription_qos_config_);
    }
    instance = it->second;
  }

  return instance->ensureRunning();
}

void VideoStreamRegistry::stopStream(const std::string & stream_key)
{
  std::shared_ptr<VideoStreamInstance> instance;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = stream_instances_.find(stream_key);
    if (it == stream_instances_.end()) {
      return;
    }
    instance = std::move(it->second);
    stream_instances_.erase(it);
  }

  if (instance) {
    instance->shutdown();
  }
}

void VideoStreamRegistry::shutdown()
{
  std::unordered_map<std::string, std::shared_ptr<VideoStreamInstance>> stream_instances;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    is_shutdown_ = true;
    stream_instances = std::move(stream_instances_);
    stream_instances_.clear();
  }

  for (auto & entry : stream_instances) {
    if (entry.second) {
      entry.second->shutdown();
    }
  }
}

}  // namespace livekit_ros2_bridge
