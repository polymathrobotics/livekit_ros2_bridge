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

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.access_policy");
constexpr char kMatchAllRule[] = "*";
constexpr char kPolicyNodeName[] = "livekit_ros2_bridge_access_policy";
constexpr char kPolicyNamespace[] = "/";
constexpr auto kSubtreeWildcardSize = sizeof(kRosResourceSubtreeWildcard) - 1U;

const char * accessOperationName(AccessOperation operation)
{
  switch (operation) {
    case AccessOperation::Publish:
      return "publish";
    case AccessOperation::Subscribe:
      return "subscribe";
    case AccessOperation::CallService:
      return "call_service";
  }

  return "unknown";
}

std::string normalizePolicyResourcePattern(std::string_view pattern, bool is_service)
{
  if (pattern == kRosResourceSubtreeWildcard) {
    return std::string{kRosResourceSubtreeWildcard};
  }

  if (
    pattern.size() >= kSubtreeWildcardSize &&
    pattern.substr(pattern.size() - kSubtreeWildcardSize) == kRosResourceSubtreeWildcard)
  {
    const auto prefix_pattern = pattern.substr(0, pattern.size() - kSubtreeWildcardSize);
    const auto prefix = normalizeRosResourceName(prefix_pattern, kPolicyNodeName, kPolicyNamespace, is_service);
    if (prefix.empty()) {
      return "";
    }
    return prefix + kRosResourceSubtreeWildcard;
  }

  return normalizeRosResourceName(pattern, kPolicyNodeName, kPolicyNamespace, is_service);
}

}  // namespace

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_allow_(Rules::parse(config.publish.allow, false))
, publish_deny_(Rules::parse(config.publish.deny, false))
, subscribe_allow_(Rules::parse(config.subscribe.allow, false))
, subscribe_deny_(Rules::parse(config.subscribe.deny, false))
, service_allow_(Rules::parse(config.service.allow, true))
, service_deny_(Rules::parse(config.service.deny, true))
{
  LogEvent(kLogger, "access_policy_loaded")
    .field("publish_allow_all", publish_allow_.matches_all)
    .field("publish_allow_patterns", publish_allow_.patterns.size())
    .field("publish_deny_all", publish_deny_.matches_all)
    .field("publish_deny_patterns", publish_deny_.patterns.size())
    .field("subscribe_allow_all", subscribe_allow_.matches_all)
    .field("subscribe_allow_patterns", subscribe_allow_.patterns.size())
    .field("subscribe_deny_all", subscribe_deny_.matches_all)
    .field("subscribe_deny_patterns", subscribe_deny_.patterns.size())
    .field("service_allow_all", service_allow_.matches_all)
    .field("service_allow_patterns", service_allow_.patterns.size())
    .field("service_deny_all", service_deny_.matches_all)
    .field("service_deny_patterns", service_deny_.patterns.size())
    .info();
}

bool AccessPolicy::allows(AccessOperation operation, std::string_view raw_resource) const
{
  const bool is_service = operation == AccessOperation::CallService;
  const auto resource = normalizeRosResourceName(raw_resource, kPolicyNodeName, kPolicyNamespace, is_service);
  if (resource.empty()) {
    LogEvent(kLogger, "access_check_rejected")
      .field("operation", accessOperationName(operation))
      .field("reason", "invalid_resource_name")
      .warn();
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

AccessPolicy::Rules AccessPolicy::Rules::parse(const std::vector<std::string> & raw_rules, bool is_service)
{
  Rules rules;
  for (const auto & raw_rule : raw_rules) {
    const std::string rule = trim(raw_rule);
    if (rule.empty()) {
      continue;
    }
    if (rule == kMatchAllRule) {
      rules.matches_all = true;
      continue;
    }

    const auto pattern = normalizePolicyResourcePattern(rule, is_service);
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
