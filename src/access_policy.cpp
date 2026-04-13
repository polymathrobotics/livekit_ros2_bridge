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

const auto kAccessPolicyLogger = rclcpp::get_logger("livekit_ros2_bridge.access_policy");
constexpr char kMatchAllEntry[] = "*";

const char * accessOperationLogName(AccessOperation operation)
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

bool AccessPolicy::RuleEntries::matches(std::string_view name) const
{
  return matches_all || std::any_of(patterns.begin(), patterns.end(), [name](const std::string & pattern) {
           return rosResourceMatchesPattern(name, pattern);
         });
}

AccessPolicy::RuleEntries AccessPolicy::RuleEntries::parse(const std::vector<std::string> & entries)
{
  RuleEntries rules;
  for (const auto & raw_entry : entries) {
    const std::string entry = trim(raw_entry);
    if (entry.empty()) {
      continue;
    }
    if (entry == kMatchAllEntry) {
      // `"*"` means allow or deny the entire operation. Normalizing it into `/*` would narrow
      // it to descendant matching instead of preserving the policy-wide override.
      rules.matches_all = true;
      continue;
    }

    const std::string pattern = normalizeRosResourceName(entry);
    if (pattern.empty()) {
      continue;
    }

    rules.patterns.insert(pattern);
  }

  return rules;
}

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_allow_(RuleEntries::parse(config.publish.allow))
, publish_deny_(RuleEntries::parse(config.publish.deny))
, subscribe_allow_(RuleEntries::parse(config.subscribe.allow))
, subscribe_deny_(RuleEntries::parse(config.subscribe.deny))
, service_allow_(RuleEntries::parse(config.service.allow))
, service_deny_(RuleEntries::parse(config.service.deny))
{
  // Log the effective parsed policy, not the raw config text, so startup diagnostics reflect
  // trimming, normalization, wildcard handling, and duplicate collapse.
  LogEvent(kAccessPolicyLogger, "access_policy_loaded")
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

bool AccessPolicy::allows(AccessOperation operation, std::string_view resource_name) const
{
  const std::string name = normalizeRosResourceName(resource_name);
  if (name.empty()) {
    LogEvent(kAccessPolicyLogger, "access_check_rejected")
      .field("operation", accessOperationLogName(operation))
      .field("reason", "empty_resource_name")
      .warn();
    return false;
  }

  switch (operation) {
    case AccessOperation::Publish:
      return !publish_deny_.matches(name) && publish_allow_.matches(name);
    case AccessOperation::Subscribe:
      return !subscribe_deny_.matches(name) && subscribe_allow_.matches(name);
    case AccessOperation::CallService:
      return !service_deny_.matches(name) && service_allow_.matches(name);
  }

  return false;
}
}  // namespace livekit_ros2_bridge
