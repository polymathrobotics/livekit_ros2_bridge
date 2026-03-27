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

#include "livekit_ros2_bridge/access_policy.hpp"

#include <algorithm>
#include <cctype>

namespace livekit_ros2_bridge
{

namespace
{

std::string trim(std::string_view value)
{
  const auto begin =
    std::find_if_not(value.begin(), value.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
  const auto end =
    std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

}  // namespace

std::string normalizeRosResourceName(std::string_view name)
{
  const std::string trimmed = trim(name);
  if (trimmed.empty()) {
    return "";
  }

  std::string normalized;
  normalized.reserve(trimmed.size() + 1);
  bool previous_was_slash = false;
  for (char ch : trimmed) {
    if (ch == '/') {
      if (!previous_was_slash) {
        normalized.push_back(ch);
      }
      previous_was_slash = true;
      continue;
    }
    normalized.push_back(ch);
    previous_was_slash = false;
  }

  if (normalized.empty()) {
    return "";
  }
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

bool accessEntryMatches(std::string_view name, std::string_view entry)
{
  if (entry.size() >= 2 && entry.substr(entry.size() - 2) == "/*") {
    const std::string prefix(entry.substr(0, entry.size() - 2));
    return name.rfind(prefix + "/", 0) == 0;
  }
  return name == entry;
}

StaticAccessPolicy::StaticAccessPolicy(
  const std::vector<std::string> & subscribe_allow,
  const std::vector<std::string> & subscribe_deny,
  const std::vector<std::string> & service_allow,
  const std::vector<std::string> & service_deny)
: subscribe_allow_(parseAllowlist(subscribe_allow))
, subscribe_deny_(parseDenylist(subscribe_deny))
, service_allow_(parseAllowlist(service_allow))
, service_deny_(parseDenylist(service_deny))
{}

bool StaticAccessPolicy::authorize(AccessOperation op, std::string_view name) const
{
  const std::string normalized = normalizeRosResourceName(name);
  if (normalized.empty()) {
    return false;
  }

  switch (op) {
    case AccessOperation::Subscribe:
      return isAllowed(normalized, subscribe_allow_.allow_all, subscribe_allow_.patterns, subscribe_deny_);
    case AccessOperation::CallService:
      return isAllowed(normalized, service_allow_.allow_all, service_allow_.patterns, service_deny_);
  }

  return false;
}

StaticAccessPolicy::ParsedPatterns StaticAccessPolicy::parseAllowlist(const std::vector<std::string> & entries)
{
  ParsedPatterns parsed;
  for (const auto & entry : entries) {
    const std::string token = trim(entry);
    if (token.empty()) {
      continue;
    }
    if (token == "*") {
      parsed.allow_all = true;
      continue;
    }
    const std::string normalized = normalizeRosResourceName(token);
    if (!normalized.empty()) {
      parsed.patterns.insert(normalized);
    }
  }
  return parsed;
}

std::set<std::string> StaticAccessPolicy::parseDenylist(const std::vector<std::string> & entries)
{
  std::set<std::string> parsed;
  for (const auto & entry : entries) {
    const std::string token = trim(entry);
    if (token.empty()) {
      continue;
    }
    if (token == "*") {
      parsed.insert("/*");
      continue;
    }
    const std::string normalized = normalizeRosResourceName(token);
    if (!normalized.empty()) {
      parsed.insert(normalized);
    }
  }
  return parsed;
}

bool StaticAccessPolicy::matchesAny(std::string_view name, const std::set<std::string> & entries)
{
  return std::any_of(
    entries.begin(), entries.end(), [name](const std::string & entry) { return accessEntryMatches(name, entry); });
}

bool StaticAccessPolicy::isAllowed(
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
