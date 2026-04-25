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

#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kNodeName[] = "livekit_ros2_bridge_access_policy";
constexpr char kRosNamespace[] = "/";

}  // namespace

AccessPolicy::AccessPolicy(const AccessPolicyConfig & config)
: publish_allow_(Rules::parse(config.publish.allow, false))
, publish_deny_(Rules::parse(config.publish.deny, false))
, subscribe_allow_(Rules::parse(config.subscribe.allow, false))
, subscribe_deny_(Rules::parse(config.subscribe.deny, false))
, service_allow_(Rules::parse(config.service.allow, true))
, service_deny_(Rules::parse(config.service.deny, true))
{}

bool AccessPolicy::allows(AccessOperation operation, std::string_view name) const
{
  const bool is_service = operation == AccessOperation::CallService;
  const auto resource = normalizeRosResourceName(name, kNodeName, kRosNamespace, is_service);
  if (resource.empty()) {
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

AccessPolicy::Rules AccessPolicy::Rules::parse(const std::vector<std::string> & rule_entries, bool is_service)
{
  Rules rules;
  for (const auto & entry : rule_entries) {
    const auto pattern = RosResourcePattern::parse(entry, kNodeName, kRosNamespace, is_service);
    if (!pattern.has_value()) {
      continue;
    }

    rules.patterns.push_back(*pattern);
  }

  return rules;
}

bool AccessPolicy::Rules::matches(std::string_view resource) const
{
  return std::any_of(patterns.begin(), patterns.end(), [resource](const RosResourcePattern & pattern) {
    return pattern.matches(resource);
  });
}
}  // namespace livekit_ros2_bridge
