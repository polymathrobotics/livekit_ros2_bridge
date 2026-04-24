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

namespace ros_resource_name_utils_detail
{
constexpr char kResourceNameExpansionNode[] = "livekit_ros2_bridge_resource_name";
constexpr char kResourceNameExpansionNamespace[] = "/";
}  // namespace ros_resource_name_utils_detail

/// Expand and validate a ROS topic/resource name for policy and protocol comparisons.
/// Surrounding whitespace is trimmed before delegating to ROS name expansion. Relative names
/// resolve from the root namespace to preserve this bridge's existing public canonical form.
/// Returns an empty string when the trimmed input is empty or ROS rejects the name.
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

inline std::string normalizeRosResourceName(std::string_view name)
{
  return normalizeRosResourceName(
    name,
    ros_resource_name_utils_detail::kResourceNameExpansionNode,
    ros_resource_name_utils_detail::kResourceNameExpansionNamespace);
}

/// Match normalized names against normalized policy patterns.
/// Exact patterns match only the same resource; patterns ending in `/*` match descendants under
/// that prefix, with `/*` acting as the root-subtree wildcard.
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
