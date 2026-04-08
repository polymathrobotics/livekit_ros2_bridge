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

#include <string>
#include <string_view>

#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

/// Normalize a ROS resource name by trimming whitespace, collapsing repeated slashes,
/// ensuring a leading slash, and removing trailing slashes except for the root "/".
inline std::string normalizeRosResourceName(std::string_view name)
{
  const std::string trimmed = trim(name);
  if (trimmed.empty()) {
    return "";
  }

  std::string normalized;
  normalized.reserve(trimmed.size() + 1);
  bool previous_was_slash = false;
  for (char ch : trimmed) {
    if (ch == '/') {
      if (!previous_was_slash) {
        normalized.push_back(ch);
      }
      previous_was_slash = true;
      continue;
    }
    normalized.push_back(ch);
    previous_was_slash = false;
  }

  if (normalized.empty()) {
    return "";
  }
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

/// Match a normalized ROS resource name against either an exact pattern ("/camera")
/// or a subtree pattern ending in "/*" ("/camera/*").
inline bool rosResourceMatchesPattern(std::string_view name, std::string_view pattern)
{
  if (pattern.size() >= 2 && pattern.substr(pattern.size() - 2) == "/*") {
    const std::string prefix(pattern.substr(0, pattern.size() - 2));
    return name.rfind(prefix + "/", 0) == 0;
  }
  return name == pattern;
}

}  // namespace livekit_ros2_bridge
