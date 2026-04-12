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

#include "video_config.hpp"

namespace livekit_ros2_bridge
{

VideoConfig makeDefaultVideoConfig()
{
  VideoConfig config;

  RosTopicRule default_rule;
  default_rule.pattern = "/*";
  default_rule.id = video_defaults::kDefaultRosProfileId;
  default_rule.transform = video_defaults::kDefaultRosTransform;
  default_rule.publish = config.publish;
  config.ros_topic_rules.push_back(std::move(default_rule));

  return config;
}

}  // namespace livekit_ros2_bridge
