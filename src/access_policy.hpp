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

struct AccessRuleConfig
{
  std::vector<std::string> allow;
  std::vector<std::string> deny;
};

struct AccessPolicyConfig
{
  AccessRuleConfig publish;
  AccessRuleConfig subscribe;
  AccessRuleConfig service;
};

enum class AccessOperation
{
  Publish,
  Subscribe,
  CallService,
};

/// Default-deny ROS topic/service policy: `"*"` is operation-wide, `/*` is
/// subtree matching, and deny rules win. Configured names are normalized at
/// construction; the result is immutable and thread-safe to share.
class AccessPolicy
{
public:
  AccessPolicy() = default;
  explicit AccessPolicy(const AccessPolicyConfig & config);

  bool allows(AccessOperation operation, std::string_view raw_resource) const;

private:
  struct Rules
  {
    static Rules parse(const std::vector<std::string> & raw_rules, bool is_service);

    bool matches_all = false;
    std::set<std::string> patterns;

    bool matches(std::string_view resource) const;
  };

  Rules publish_allow_;
  Rules publish_deny_;
  Rules subscribe_allow_;
  Rules subscribe_deny_;
  Rules service_allow_;
  Rules service_deny_;
};

}  // namespace livekit_ros2_bridge
