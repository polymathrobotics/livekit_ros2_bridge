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
constexpr char kMatchAllRuleEntry[] = "*";

}  // namespace

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_rules_(parseRuleset(config.publish.allow, config.publish.deny))
, subscribe_rules_(parseRuleset(config.subscribe.allow, config.subscribe.deny))
, service_rules_(parseRuleset(config.service.allow, config.service.deny))
{
  LogEvent(kAccessPolicyLogger, "access_policy_loaded")
    .field("phase", "startup")
    .field("publish_allow_all", publish_rules_.allow.matches_all)
    .field("publish_allow_patterns", publish_rules_.allow.patterns.size())
    .field("publish_deny_all", publish_rules_.deny.matches_all)
    .field("publish_deny_patterns", publish_rules_.deny.patterns.size())
    .field("subscribe_allow_all", subscribe_rules_.allow.matches_all)
    .field("subscribe_allow_patterns", subscribe_rules_.allow.patterns.size())
    .field("subscribe_deny_all", subscribe_rules_.deny.matches_all)
    .field("subscribe_deny_patterns", subscribe_rules_.deny.patterns.size())
    .field("service_allow_all", service_rules_.allow.matches_all)
    .field("service_allow_patterns", service_rules_.allow.patterns.size())
    .field("service_deny_all", service_rules_.deny.matches_all)
    .field("service_deny_patterns", service_rules_.deny.patterns.size())
    .info();
}

bool AccessPolicy::allows(AccessOperation op, std::string_view name) const
{
  const std::string normalized = normalizeRosResourceName(name);
  if (normalized.empty()) {
    return false;
  }

  switch (op) {
    case AccessOperation::Publish:
      return isAllowed(normalized, publish_rules_);
    case AccessOperation::Subscribe:
      return isAllowed(normalized, subscribe_rules_);
    case AccessOperation::CallService:
      return isAllowed(normalized, service_rules_);
  }

  return false;
}

AccessPolicy::ParsedRuleEntries AccessPolicy::parseRuleEntries(const std::vector<std::string> & entries)
{
  ParsedRuleEntries parsed;
  for (const auto & entry : entries) {
    const std::string token = trim(entry);
    if (token.empty()) {
      continue;
    }
    if (token == kMatchAllRuleEntry) {
      parsed.matches_all = true;
      continue;
    }

    const std::string normalized = normalizeRosResourceName(token);
    if (!normalized.empty()) {
      parsed.patterns.insert(normalized);
    }
  }

  return parsed;
}

AccessPolicy::ParsedRuleset AccessPolicy::parseRuleset(
  const std::vector<std::string> & allow_entries, const std::vector<std::string> & deny_entries)
{
  ParsedRuleset parsed;
  parsed.allow = parseRuleEntries(allow_entries);
  parsed.deny = parseRuleEntries(deny_entries);
  return parsed;
}

bool AccessPolicy::isAllowed(std::string_view name, const ParsedRuleset & ruleset)
{
  const auto matches = [name](const ParsedRuleEntries & entries) {
    return std::any_of(entries.patterns.begin(), entries.patterns.end(), [name](const std::string & entry) {
      return rosResourceMatchesPattern(name, entry);
    });
  };

  const bool deny_rule_matched = ruleset.deny.matches_all || matches(ruleset.deny);
  if (deny_rule_matched) {
    return false;
  }
  if (ruleset.allow.matches_all) {
    return true;
  }
  if (ruleset.allow.patterns.empty()) {
    return false;
  }
  return matches(ruleset.allow);
}

}  // namespace livekit_ros2_bridge
