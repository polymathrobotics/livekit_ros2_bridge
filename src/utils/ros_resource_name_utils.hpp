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

/// Canonicalize a ROS topic/service/resource name for policy and protocol comparisons:
/// trim surrounding whitespace, collapse repeated `/`, prepend a leading `/` when missing,
/// and drop trailing `/` except for the root name `/`. Returns an empty string only when the
/// trimmed input is empty.
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

  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

/// Match normalized names against normalized policy patterns.
/// Exact patterns match only the same resource; patterns ending in `/*` match descendants under
/// that prefix, with `/*` acting as the root-subtree wildcard.
inline bool rosResourceMatchesPattern(std::string_view name, std::string_view pattern)
{
  const bool is_subtree_pattern = pattern.size() >= 2 && pattern.substr(pattern.size() - 2) == "/*";
  if (is_subtree_pattern) {
    const std::string prefix(pattern.substr(0, pattern.size() - 2));
    return name.rfind(prefix + "/", 0) == 0;
  }
  return name == pattern;
}

}  // namespace livekit_ros2_bridge
