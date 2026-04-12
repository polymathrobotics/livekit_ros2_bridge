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

#include "video_stream_instance.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "subscription_qos.hpp"
#include "video_frame_source.hpp"
#include "video_track_publisher.hpp"

namespace livekit_ros2_bridge
{

VideoStreamInstance::VideoStreamInstance(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  VideoStreamSpec spec,
  const SubscriptionQosConfig * subscription_qos_config)
: node_(node)
, spec_(std::move(spec))
, subscription_qos_config_(subscription_qos_config)
, video_track_publisher_(std::make_unique<VideoTrackPublisher>(room_connection, spec_))
{}

VideoStreamInstance::~VideoStreamInstance()
{
  shutdown();
}

std::string VideoStreamInstance::ensureRunning()
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

void VideoStreamInstance::shutdown()
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

std::shared_ptr<VideoFrameSource> VideoStreamInstance::createFrameSourceLocked()
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

}  // namespace livekit_ros2_bridge
