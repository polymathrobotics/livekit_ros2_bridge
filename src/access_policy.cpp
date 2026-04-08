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

namespace
{

template <typename WildcardHandler, typename NormalizedEntryHandler>
void parseAccessEntries(
  const std::vector<std::string> & entries,
  WildcardHandler && wildcard_handler,
  NormalizedEntryHandler && normalized_entry_handler)
{
  for (const auto & entry : entries) {
    const std::string token = trim(entry);
    if (token.empty()) {
      continue;
    }
    if (token == "*") {
      wildcard_handler();
      continue;
    }
    const std::string normalized = normalizeRosResourceName(token);
    if (!normalized.empty()) {
      normalized_entry_handler(normalized);
    }
  }
}

}  // namespace

AccessPolicy::AccessPolicy(
  const std::vector<std::string> & publish_allow,
  const std::vector<std::string> & publish_deny,
  const std::vector<std::string> & subscribe_allow,
  const std::vector<std::string> & subscribe_deny,
  const std::vector<std::string> & service_allow,
  const std::vector<std::string> & service_deny)
: publish_allow_(parseAllowlist(publish_allow))
, publish_deny_(parseDenylist(publish_deny))
, subscribe_allow_(parseAllowlist(subscribe_allow))
, subscribe_deny_(parseDenylist(subscribe_deny))
, service_allow_(parseAllowlist(service_allow))
, service_deny_(parseDenylist(service_deny))
{}

bool AccessPolicy::allows(AccessOperation op, std::string_view name) const
{
  const std::string normalized = normalizeRosResourceName(name);
  if (normalized.empty()) {
    return false;
  }

  switch (op) {
    case AccessOperation::Publish:
      return isAllowed(normalized, publish_allow_.allow_all, publish_allow_.patterns, publish_deny_);
    case AccessOperation::Subscribe:
      return isAllowed(normalized, subscribe_allow_.allow_all, subscribe_allow_.patterns, subscribe_deny_);
    case AccessOperation::CallService:
      return isAllowed(normalized, service_allow_.allow_all, service_allow_.patterns, service_deny_);
  }

  return false;
}

AccessPolicy::ParsedAllowlist AccessPolicy::parseAllowlist(const std::vector<std::string> & entries)
{
  ParsedAllowlist parsed;
  parseAccessEntries(
    entries,
    [&parsed]() { parsed.allow_all = true; },
    [&parsed](const std::string & normalized) { parsed.patterns.insert(normalized); });
  return parsed;
}

std::set<std::string> AccessPolicy::parseDenylist(const std::vector<std::string> & entries)
{
  std::set<std::string> parsed;
  parseAccessEntries(
    entries,
    [&parsed]() { parsed.insert("/*"); },
    [&parsed](const std::string & normalized) { parsed.insert(normalized); });
  return parsed;
}

bool AccessPolicy::matchesAny(std::string_view name, const std::set<std::string> & entries)
{
  return std::any_of(entries.begin(), entries.end(), [name](const std::string & entry) {
    return rosResourceMatchesPattern(name, entry);
  });
}

bool AccessPolicy::isAllowed(
  std::string_view name,
  bool allow_all,
  const std::set<std::string> & allowlist,
  const std::set<std::string> & denylist)
{
  if (allow_all) {
    return !matchesAny(name, denylist);
  }
  if (allowlist.empty()) {
    return false;
  }
  if (matchesAny(name, denylist)) {
    return false;
  }
  return matchesAny(name, allowlist);
}

}  // namespace livekit_ros2_bridge
