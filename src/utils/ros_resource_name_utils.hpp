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

#include <stdexcept>
#include <string>
#include <string_view>

#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

constexpr char kRosResourceSubtreeWildcard[] = "/*";

/// Returns empty for blank input or ROS name validation/expansion failures.
inline std::string normalizeRosResourceName(
  std::string_view name, std::string_view node_name, std::string_view namespace_, bool is_service = false)
{
  const std::string trimmed = trim(name);
  if (trimmed.empty()) {
    return "";
  }

  try {
    return rclcpp::expand_topic_or_service_name(trimmed, std::string(node_name), std::string(namespace_), is_service);
  } catch (const rclcpp::exceptions::NameValidationError &) {
    return "";
  } catch (const std::runtime_error &) {
    return "";
  }
}

/// Canonicalizes config/protocol names by resolving relatives from `/`.
inline std::string normalizeRosResourceName(std::string_view name)
{
  return normalizeRosResourceName(name, "livekit_ros2_bridge_resource_name", "/");
}

/// A terminal `/*` matches names under that prefix; `/*` also matches `/`.
inline bool rosResourceMatchesPattern(std::string_view name, std::string_view pattern)
{
  const bool is_subtree_pattern =
    pattern.size() >= sizeof(kRosResourceSubtreeWildcard) - 1U &&
    pattern.substr(pattern.size() - (sizeof(kRosResourceSubtreeWildcard) - 1U)) == kRosResourceSubtreeWildcard;
  if (is_subtree_pattern) {
    const std::string prefix(pattern.substr(0, pattern.size() - (sizeof(kRosResourceSubtreeWildcard) - 1U)));
    return name.rfind(prefix + "/", 0) == 0;
  }
  return name == pattern;
}

}  // namespace livekit_ros2_bridge
