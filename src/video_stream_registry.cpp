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

#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "video_stream_instance.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.video_stream_registry");

}  // namespace

VideoStreamRegistry::VideoStreamRegistry(
  rclcpp::Node & node,
  RoomConnection & room_connection,
  const SubscriptionQosConfig * subscription_qos_config,
  VideoProfilingRegistry * profiling_registry)
: node_(node)
, room_connection_(room_connection)
, subscription_qos_config_(subscription_qos_config)
, profiling_registry_(profiling_registry)
{}

VideoStreamRegistry::~VideoStreamRegistry()
{
  shutdown();
}

std::string VideoStreamRegistry::start(const VideoStreamSpec & spec)
{
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
      instance =
        std::make_shared<VideoStreamInstance>(node_, room_connection_, spec, subscription_qos_config_, profiler);
      instances_.emplace(spec.stream_key, instance);
    }
  }

  // Start outside the registry mutex; VideoStreamInstance owns the heavier startup path and its
  // lifecycle locking.
  return instance->start();
}

void VideoStreamRegistry::stop(const std::string & stream_key)
{
  std::shared_ptr<VideoStreamInstance> instance;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = instances_.find(stream_key);
    if (it != instances_.end()) {
      instance = std::move(it->second);
      instances_.erase(it);
    }
  }
  if (instance == nullptr) {
    return;
  }

  // Erase first so a concurrent restart gets a fresh registry entry. The detached shared_ptr keeps
  // the instance alive through teardown.
  instance->shutdown();
}

void VideoStreamRegistry::shutdown()
{
  std::vector<std::shared_ptr<VideoStreamInstance>> instances;
  std::size_t stream_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    // Mark shutdown before detaching entries so concurrent `start()` calls fail during teardown.
    is_shutdown_ = true;
    stream_count = instances_.size();
    instances.reserve(stream_count);
    for (auto & entry : instances_) {
      instances.push_back(std::move(entry.second));
    }
    instances_.clear();
  }

  if (stream_count > 0) {
    LogEvent(kLogger, "video_stream_registry_shutdown_begin").field("stream_count", stream_count).info();
  }

  // Preserve ownership after clearing the map because shutdown touches external ROS and LiveKit
  // state.
  for (auto & instance : instances) {
    instance->shutdown();
  }
}

}  // namespace livekit_ros2_bridge
