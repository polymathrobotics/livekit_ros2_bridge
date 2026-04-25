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

#include "access_policy.hpp"

#include <algorithm>

#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kMatchAllRule[] = "*";
constexpr char kNodeName[] = "livekit_ros2_bridge_access_policy";
constexpr char kRosNamespace[] = "/";
constexpr auto kSubtreeWildcardSize = sizeof(kRosResourceSubtreeWildcard) - 1U;

std::string normalizePattern(std::string_view pattern, bool is_service)
{
  if (pattern == kRosResourceSubtreeWildcard) {
    return std::string{kRosResourceSubtreeWildcard};
  }

  if (
    pattern.size() >= kSubtreeWildcardSize &&
    pattern.substr(pattern.size() - kSubtreeWildcardSize) == kRosResourceSubtreeWildcard)
  {
    const auto prefix = normalizeRosResourceName(
      pattern.substr(0, pattern.size() - kSubtreeWildcardSize), kNodeName, kRosNamespace, is_service);
    if (prefix.empty()) {
      return "";
    }
    return prefix + kRosResourceSubtreeWildcard;
  }

  return normalizeRosResourceName(pattern, kNodeName, kRosNamespace, is_service);
}

}  // namespace

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_allow_(Rules::parse(config.publish.allow, false))
, publish_deny_(Rules::parse(config.publish.deny, false))
, subscribe_allow_(Rules::parse(config.subscribe.allow, false))
, subscribe_deny_(Rules::parse(config.subscribe.deny, false))
, service_allow_(Rules::parse(config.service.allow, true))
, service_deny_(Rules::parse(config.service.deny, true))
{}

bool AccessPolicy::allows(AccessOperation operation, std::string_view name) const
{
  const bool is_service = operation == AccessOperation::CallService;
  const auto resource = normalizeRosResourceName(name, kNodeName, kRosNamespace, is_service);
  if (resource.empty()) {
    return false;
  }

  switch (operation) {
    case AccessOperation::Publish:
      return !publish_deny_.matches(resource) && publish_allow_.matches(resource);
    case AccessOperation::Subscribe:
      return !subscribe_deny_.matches(resource) && subscribe_allow_.matches(resource);
    case AccessOperation::CallService:
      return !service_deny_.matches(resource) && service_allow_.matches(resource);
  }

  return false;
}

AccessPolicy::Rules AccessPolicy::Rules::parse(const std::vector<std::string> & rule_entries, bool is_service)
{
  Rules rules;
  for (const auto & entry : rule_entries) {
    const std::string rule = trim(entry);
    if (rule.empty()) {
      continue;
    }
    if (rule == kMatchAllRule) {
      rules.matches_all = true;
      continue;
    }

    const auto pattern = normalizePattern(rule, is_service);
    if (pattern.empty()) {
      continue;
    }

    rules.patterns.insert(pattern);
  }

  return rules;
}

bool AccessPolicy::Rules::matches(std::string_view resource) const
{
  return matches_all || std::any_of(patterns.begin(), patterns.end(), [resource](const std::string & pattern) {
           return rosResourceMatchesPattern(resource, pattern);
         });
}
}  // namespace livekit_ros2_bridge
