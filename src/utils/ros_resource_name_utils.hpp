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

#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

class RosResourcePattern
{
public:
  enum class Kind
  {
    Exact,
    Subtree,
  };

  RosResourcePattern() = default;

  static std::optional<RosResourcePattern> parse(
    std::string_view raw, std::string_view node_name, std::string_view namespace_, bool is_service = false)
  {
    const std::string pattern = trim(raw);
    if (pattern.empty()) {
      return std::nullopt;
    }
    if (pattern == "*") {
      return rootSubtree();
    }
    if (pattern == kRosResourceSubtreeWildcard) {
      return rootSubtree();
    }

    if (hasSubtreeWildcard(pattern)) {
      const auto suffix_size = subtreeWildcardSize();
      const std::string prefix = normalizeRosResourceName(
        std::string_view(pattern).substr(0, pattern.size() - suffix_size), node_name, namespace_, is_service);
      if (prefix.empty()) {
        return std::nullopt;
      }
      return RosResourcePattern(prefix + kRosResourceSubtreeWildcard, Kind::Subtree);
    }

    const std::string resource = normalizeRosResourceName(pattern, node_name, namespace_, is_service);
    if (resource.empty()) {
      return std::nullopt;
    }
    return RosResourcePattern(resource, Kind::Exact);
  }

  /// Canonicalizes config/protocol patterns by resolving relatives from `/`.
  static std::optional<RosResourcePattern> parse(std::string_view raw)
  {
    return parse(raw, "livekit_ros2_bridge_resource_name", "/");
  }

  /// Wraps a pattern that has already been normalized by this helper.
  static RosResourcePattern fromCanonical(std::string_view pattern)
  {
    if (pattern == "*" || pattern == kRosResourceSubtreeWildcard) {
      return rootSubtree();
    }
    return RosResourcePattern(std::string(pattern), hasSubtreeWildcard(pattern) ? Kind::Subtree : Kind::Exact);
  }

  static RosResourcePattern rootSubtree()
  {
    return RosResourcePattern(std::string{kRosResourceSubtreeWildcard}, Kind::Subtree);
  }

  std::string_view canonical() const noexcept
  {
    return canonical_;
  }

  std::size_t specificity() const noexcept
  {
    return canonical_.size();
  }

  Kind kind() const noexcept
  {
    return kind_;
  }

  /// A terminal `/*` matches names under that prefix; `/*` also matches `/`.
  bool matches(std::string_view resource) const noexcept
  {
    if (kind_ == Kind::Exact) {
      return resource == canonical_;
    }

    const std::string_view prefix{canonical_.data(), canonical_.size() - subtreeWildcardSize()};
    if (prefix.empty()) {
      return resource.rfind("/", 0) == 0;
    }
    return resource.size() > prefix.size() && resource.compare(0, prefix.size(), prefix) == 0 &&
           resource[prefix.size()] == '/';
  }

private:
  static constexpr std::size_t subtreeWildcardSize() noexcept
  {
    return sizeof(kRosResourceSubtreeWildcard) - 1U;
  }

  static bool hasSubtreeWildcard(std::string_view pattern) noexcept
  {
    const auto suffix_size = subtreeWildcardSize();
    return pattern.size() >= suffix_size && pattern.substr(pattern.size() - suffix_size) == kRosResourceSubtreeWildcard;
  }

  RosResourcePattern(std::string canonical, Kind kind)
  : canonical_(std::move(canonical))
  , kind_(kind)
  {}

  std::string canonical_;
  Kind kind_ = Kind::Exact;
};

inline bool operator==(const RosResourcePattern & lhs, std::string_view rhs) noexcept
{
  return lhs.canonical() == rhs;
}

inline bool operator==(std::string_view lhs, const RosResourcePattern & rhs) noexcept
{
  return lhs == rhs.canonical();
}

inline bool operator!=(const RosResourcePattern & lhs, std::string_view rhs) noexcept
{
  return !(lhs == rhs);
}

inline bool operator!=(std::string_view lhs, const RosResourcePattern & rhs) noexcept
{
  return !(lhs == rhs);
}

inline std::ostream & operator<<(std::ostream & stream, const RosResourcePattern & pattern)
{
  return stream << pattern.canonical();
}

template <typename Entry>
const Entry * findBestRosResourcePatternMatch(
  const std::vector<Entry> & entries, std::string_view resource, const RosResourcePattern Entry::* pattern_member)
{
  const Entry * match = nullptr;
  for (const auto & entry : entries) {
    const auto & pattern = entry.*pattern_member;
    if (!pattern.matches(resource)) {
      continue;
    }
    if (match != nullptr && pattern.specificity() <= (match->*pattern_member).specificity()) {
      continue;
    }
    match = &entry;
  }
  return match;
}

inline bool rosResourceMatchesPattern(std::string_view resource, std::string_view pattern)
{
  return RosResourcePattern::fromCanonical(pattern).matches(resource);
}

}  // namespace livekit_ros2_bridge
