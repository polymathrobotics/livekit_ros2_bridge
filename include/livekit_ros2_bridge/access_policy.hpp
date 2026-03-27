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

namespace livekit_ros2_bridge
{

enum class AccessOperation
{
  Subscribe,
  CallService,
};

std::string normalizeRosResourceName(std::string_view name);
bool accessEntryMatches(std::string_view name, std::string_view entry);

class StaticAccessPolicy
{
public:
  StaticAccessPolicy() = default;
  StaticAccessPolicy(
    const std::vector<std::string> & subscribe_allow,
    const std::vector<std::string> & subscribe_deny,
    const std::vector<std::string> & service_allow,
    const std::vector<std::string> & service_deny);

  bool authorize(AccessOperation op, std::string_view name) const;

private:
  struct ParsedPatterns
  {
    bool allow_all = false;
    std::set<std::string> patterns;
  };

  static ParsedPatterns parseAllowlist(const std::vector<std::string> & entries);
  static std::set<std::string> parseDenylist(const std::vector<std::string> & entries);
  static bool matchesAny(std::string_view name, const std::set<std::string> & entries);
  static bool isAllowed(
    std::string_view name,
    bool allow_all,
    const std::set<std::string> & allowlist,
    const std::set<std::string> & denylist);

  ParsedPatterns subscribe_allow_;
  std::set<std::string> subscribe_deny_;
  ParsedPatterns service_allow_;
  std::set<std::string> service_deny_;
};

}  // namespace livekit_ros2_bridge
