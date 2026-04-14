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
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.access_policy");
constexpr char kMatchAllRule[] = "*";

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

}  // namespace

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_allow_(Rules::parse(config.publish.allow))
, publish_deny_(Rules::parse(config.publish.deny))
, subscribe_allow_(Rules::parse(config.subscribe.allow))
, subscribe_deny_(Rules::parse(config.subscribe.deny))
, service_allow_(Rules::parse(config.service.allow))
, service_deny_(Rules::parse(config.service.deny))
{
  // Log the effective parsed policy, not the raw config text, so startup diagnostics reflect
  // trimming, normalization, wildcard handling, and duplicate collapse.
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
  const std::string resource = normalizeRosResourceName(raw_resource);
  if (resource.empty()) {
    LogEvent(kLogger, "access_check_rejected")
      .field("operation", accessOperationName(operation))
      .field("reason", "empty_resource_name")
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

AccessPolicy::Rules AccessPolicy::Rules::parse(const std::vector<std::string> & raw_rules)
{
  Rules rules;
  for (const auto & raw_rule : raw_rules) {
    const std::string rule = trim(raw_rule);
    if (rule.empty()) {
      continue;
    }
    if (rule == kMatchAllRule) {
      // `"*"` means allow or deny the entire operation. Normalizing it into `/*` would narrow
      // it to descendant matching instead of preserving the policy-wide override.
      rules.matches_all = true;
      continue;
    }

    const std::string pattern = normalizeRosResourceName(rule);
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
