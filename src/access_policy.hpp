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

enum class AccessOperation
{
  Publish,
  Subscribe,
  CallService,
};

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

  bool allows(AccessOperation op, std::string_view name) const;

private:
  struct ParsedAllowlist
  {
    bool allow_all = false;
    std::set<std::string> patterns;
  };

  static ParsedAllowlist parseAllowlist(const std::vector<std::string> & entries);
  static std::set<std::string> parseDenylist(const std::vector<std::string> & entries);
  static bool matchesAny(std::string_view name, const std::set<std::string> & entries);
  static bool isAllowed(
    std::string_view name,
    bool allow_all,
    const std::set<std::string> & allowlist,
    const std::set<std::string> & denylist);

  ParsedAllowlist publish_allow_;
  std::set<std::string> publish_deny_;
  ParsedAllowlist subscribe_allow_;
  std::set<std::string> subscribe_deny_;
  ParsedAllowlist service_allow_;
  std::set<std::string> service_deny_;
};

}  // namespace livekit_ros2_bridge
