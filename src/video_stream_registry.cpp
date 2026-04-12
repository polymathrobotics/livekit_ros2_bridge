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

#include <string>

#include "video_stream_instance.hpp"

namespace livekit_ros2_bridge
{

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
      const std::shared_ptr<VideoStreamProfiler> profiler =
        profiling_registry_ == nullptr ? nullptr : profiling_registry_->getOrCreateStreamProfiler(spec);
      it->second =
        std::make_shared<VideoStreamInstance>(node_, room_connection_, spec, subscription_qos_config_, profiler);
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
