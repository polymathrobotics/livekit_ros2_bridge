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

#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

/// Resource access categories enforced by the bridge.
enum class AccessOperation
{
  Publish,
  Subscribe,
  CallService,
};

/// Operation-specific allow/deny rules over normalized ROS resource names.
/// The policy is default-deny, a literal `"*"` allow entry means allow all for that operation,
/// and deny entries always win over allows. Entries are trimmed and normalized before matching;
/// exact patterns match one resource and `.../*` patterns match descendants.
class AccessPolicy
{
public:
  AccessPolicy() = default;
  AccessPolicy(
    const std::vector<std::string> & publish_allow,
    const std::vector<std::string> & publish_deny,
    const std::vector<std::string> & subscribe_allow,
    const std::vector<std::string> & subscribe_deny,
    const std::vector<std::string> & service_allow,
    const std::vector<std::string> & service_deny);

  /// Return whether `name` is allowed after normalization. Empty or whitespace-only names are denied.
  bool allows(AccessOperation op, std::string_view name) const;

private:
  struct ParsedRuleEntries
  {
    // True when the configured entries contained `"*"`.
    bool matches_all = false;
    // Normalized exact or subtree patterns. `"*"` is represented only by `matches_all`.
    std::set<std::string> patterns;
  };

  struct ParsedRuleset
  {
    ParsedRuleEntries allow;
    ParsedRuleEntries deny;
  };

  static ParsedRuleEntries parseRuleEntries(const std::vector<std::string> & entries);
  static ParsedRuleset parseRuleset(
    const std::vector<std::string> & allow_entries,
    const std::vector<std::string> & deny_entries);
  static bool matchesAny(std::string_view name, const std::set<std::string> & entries);
  static bool isAllowed(std::string_view name, const ParsedRuleset & ruleset);

  // Normalized publish rules. Denies always override allows.
  ParsedRuleset publish_rules_;
  // Normalized subscribe rules. Denies always override allows.
  ParsedRuleset subscribe_rules_;
  // Normalized service-call rules. Denies always override allows.
  ParsedRuleset service_rules_;
};

}  // namespace livekit_ros2_bridge
