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

#pragma once

#include <atomic>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include "access_policy.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialized_message.hpp"
#include "topic_publish_command.hpp"

namespace livekit_ros2_bridge
{

class RosTopicPublisher final
{
public:
  RosTopicPublisher(rclcpp::Node & node, AccessPolicy access_policy, int max_topics);

  void publish(const std::string & requester_identity, const TopicPublishCommand & command);

  void shutdown();

private:
  struct PublisherCacheEntry
  {
    std::string interface_type;
    std::shared_ptr<rclcpp::GenericPublisher> publisher_handle;
    std::list<std::string>::iterator lru_position;
  };

  std::string resolveTopicTypeOrThrow(const std::string & topic, const std::string & requested_interface_type) const;
  void publishWithPublisherCache(
    const std::string & topic, const std::string & interface_type, const rclcpp::SerializedMessage & serialized);
  void evictPublisher(const std::string & topic);

  rclcpp::Node & node_;
  AccessPolicy access_policy_;
  int max_topics_ = 0;
  std::atomic<bool> is_shutdown_{false};
  std::unordered_map<std::string, PublisherCacheEntry> publishers_;
  std::list<std::string> lru_topics_;
};

}  // namespace livekit_ros2_bridge
