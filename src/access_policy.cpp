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

#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

AccessPolicy::AccessPolicy(
  const std::vector<std::string> & publish_allow,
  const std::vector<std::string> & publish_deny,
  const std::vector<std::string> & subscribe_allow,
  const std::vector<std::string> & subscribe_deny,
  const std::vector<std::string> & service_allow,
  const std::vector<std::string> & service_deny)
: publish_rules_(parseRuleset(publish_allow, publish_deny))
, subscribe_rules_(parseRuleset(subscribe_allow, subscribe_deny))
, service_rules_(parseRuleset(service_allow, service_deny))
{}

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
    if (token == "*") {
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
  const std::vector<std::string> & allow_entries,
  const std::vector<std::string> & deny_entries)
{
  ParsedRuleset parsed;
  parsed.allow = parseRuleEntries(allow_entries);
  parsed.deny = parseRuleEntries(deny_entries);
  return parsed;
}

bool AccessPolicy::matchesAny(std::string_view name, const std::set<std::string> & entries)
{
  return std::any_of(entries.begin(), entries.end(), [name](const std::string & entry) {
    return rosResourceMatchesPattern(name, entry);
  });
}

bool AccessPolicy::isAllowed(std::string_view name, const ParsedRuleset & ruleset)
{
  if (ruleset.deny.matches_all || matchesAny(name, ruleset.deny.patterns)) {
    return false;
  }
  if (ruleset.allow.matches_all) {
    return true;
  }
  if (ruleset.allow.patterns.empty()) {
    return false;
  }
  return matchesAny(name, ruleset.allow.patterns);
}

}  // namespace livekit_ros2_bridge
