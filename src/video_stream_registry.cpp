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

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "room_connection.hpp"
#include "subscription_qos.hpp"
#include "utils/log_event.hpp"
#include "video_profiling.hpp"
#include "video_stream_instance.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

const VideoStreamConfig & defaultVideoStreamConfig()
{
  static const VideoStreamConfig kDefaultConfig = makeDefaultVideoStreamConfig();
  return kDefaultConfig;
}

}  // namespace

VideoStreamRegistry::VideoStreamRegistry(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  const SubscriptionQosConfig * qos_config,
  VideoProfilingRegistry * profiling_registry,
  const VideoStreamConfig * video_stream_config)
: node_(node)
, room_connection_(room_connection)
, qos_config_(qos_config)
, profiling_registry_(profiling_registry)
, video_stream_config_(video_stream_config)
{}

VideoStreamRegistry::~VideoStreamRegistry()
{
  shutdown();
}

VideoStreamInfo VideoStreamRegistry::resolve(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const
{
  const auto spec = resolveSpec(kind, name, interface_type);
  return {spec.stream_key, spec.track_name, spec.degraded_reason.value_or("")};
}

std::optional<VideoStreamInfo> VideoStreamRegistry::find(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const
{
  const auto spec = resolveSpec(kind, name, interface_type);

  std::lock_guard<std::mutex> lock(mutex_);
  if (instances_.find(spec.stream_key) == instances_.end()) {
    return std::nullopt;
  }

  return VideoStreamInfo{spec.stream_key, spec.track_name, spec.degraded_reason.value_or("")};
}

void VideoStreamRegistry::start(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type)
{
  const auto spec = resolveSpec(kind, name, interface_type);

  std::shared_ptr<VideoStreamInstance> instance;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      LogEvent(kLogger, "video_stream_registry_start_rejected")
        .field("stream_key", spec.stream_key)
        .field("reason", "shutdown")
        .warn();
      throw std::runtime_error("Video stream registry is shut down.");
    }

    const auto it = instances_.find(spec.stream_key);
    if (it != instances_.end()) {
      instance = it->second;
    } else {
      const auto profiler = profiling_registry_ == nullptr ? nullptr : profiling_registry_->getOrCreateProfiler(spec);
      instance = std::make_shared<VideoStreamInstance>(node_, room_connection_, spec, qos_config_, profiler);
      instances_.emplace(spec.stream_key, instance);
    }
  }

  // Start outside the registry mutex; VideoStreamInstance owns the heavier startup path and its
  // lifecycle locking.
  (void)instance->start();
}

void VideoStreamRegistry::stop(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type)
{
  const auto stream_key = resolveSpec(kind, name, interface_type).stream_key;

  std::shared_ptr<VideoStreamInstance> instance;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = instances_.find(stream_key);
    if (it == instances_.end()) {
      return;
    }
    instance = std::move(it->second);
    instances_.erase(it);
  }

  // Erase first so a concurrent restart gets a fresh registry entry. The detached shared_ptr keeps
  // the instance alive through teardown.
  instance->shutdown();
}

const VideoStreamConfig & VideoStreamRegistry::videoStreamConfig() const
{
  return video_stream_config_ == nullptr ? defaultVideoStreamConfig() : *video_stream_config_;
}

VideoStreamSpec VideoStreamRegistry::resolveSpec(
  SubscriptionTargetKind kind, const std::string & name, const std::string & interface_type) const
{
  switch (kind) {
    case SubscriptionTargetKind::Topic:
      return resolveRosVideoTopicSpec(videoStreamConfig(), name, interface_type);
    case SubscriptionTargetKind::ConfiguredSource:
      return resolveConfiguredVideoSourceSpec(videoStreamConfig(), name);
  }

  throw std::invalid_argument("video stream request kind is invalid");
}

void VideoStreamRegistry::shutdown()
{
  std::vector<std::shared_ptr<VideoStreamInstance>> instances;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    // Mark shutdown before detaching entries so concurrent `start()` calls fail during teardown,
    // even if there is nothing left to stop.
    is_shutdown_ = true;
    instances.reserve(instances_.size());
    for (auto & entry : instances_) {
      instances.push_back(std::move(entry.second));
    }
    instances_.clear();
  }

  if (instances.empty()) {
    return;
  }

  LogEvent(kLogger, "video_stream_registry_shutdown_begin").field("stream_count", instances.size()).info();

  // Preserve ownership after clearing the map because shutdown touches external ROS and LiveKit
  // state.
  for (auto & instance : instances) {
    instance->shutdown();
  }
}

}  // namespace livekit_ros2_bridge
