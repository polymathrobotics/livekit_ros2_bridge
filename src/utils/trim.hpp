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

#ifndef LIVEKIT_ROS2_BRIDGE__TRIM_HPP_
#define LIVEKIT_ROS2_BRIDGE__TRIM_HPP_

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace livekit_ros2_bridge
{

inline std::string trim(std::string_view text)
{
  const auto is_non_space = [](unsigned char ch) { return std::isspace(ch) == 0; };
  const auto first = std::find_if(text.begin(), text.end(), is_non_space);
  const auto last = std::find_if(text.rbegin(), text.rend(), is_non_space).base();

  if (first >= last) {
    return "";
  }

  return std::string(first, last);
}

}  // namespace livekit_ros2_bridge

#endif  // LIVEKIT_ROS2_BRIDGE__TRIM_HPP_
