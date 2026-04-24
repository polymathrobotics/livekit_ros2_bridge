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

#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "gtest/gtest.h"

namespace livekit_ros2_bridge
{

namespace
{

AccessPolicy makeSubscribePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.subscribe.allow = std::move(allow);
  config.subscribe.deny = std::move(deny);
  return AccessPolicy(config);
}

}  // namespace

TEST(AccessPolicyTest, DefaultPolicyDeniesEveryOperation)
{
  const AccessPolicy policy;

  EXPECT_FALSE(policy.allows(AccessOperation::Publish, "/cmd_vel"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_FALSE(policy.allows(AccessOperation::CallService, "/reset"));
}

TEST(AccessPolicyTest, UsesOperationSpecificRulesAndDenylistPrecedence)
{
  AccessPolicyConfig config;
  config.publish.allow = {"/cmd/*"};
  config.publish.deny = {"/cmd/blocked"};
  config.subscribe.allow = {"/camera/image"};
  config.service.allow = {"/example/service"};
  const AccessPolicy policy(config);

  EXPECT_TRUE(policy.allows(AccessOperation::Publish, "/cmd/allowed"));
  EXPECT_FALSE(policy.allows(AccessOperation::Publish, "/cmd/blocked"));

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/cmd/allowed"));

  EXPECT_TRUE(policy.allows(AccessOperation::CallService, "/example/service"));
  EXPECT_FALSE(policy.allows(AccessOperation::CallService, "/cmd/allowed"));
}

TEST(AccessPolicyTest, AllowAllStillHonorsDenylistEntries)
{
  const AccessPolicy exact_deny = makeSubscribePolicy({"*"}, {"/blocked"});
  const AccessPolicy wildcard_deny = makeSubscribePolicy({"*"}, {"*"});

  EXPECT_TRUE(exact_deny.allows(AccessOperation::Subscribe, "/ok"));
  EXPECT_FALSE(exact_deny.allows(AccessOperation::Subscribe, "/blocked"));
  EXPECT_FALSE(wildcard_deny.allows(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, ExpandsRelativeConfiguredEntriesAndRequestedNames)
{
  const AccessPolicy policy =
    makeSubscribePolicy({"  camera/front/*  ", "/camera/front/image"}, {"  camera/front/blocked  "});

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/front/stream"));
  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "  camera/front/image  "));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/front/blocked"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, " \t\n "));
}

TEST(AccessPolicyTest, DeniesResourceNamesRejectedByRosValidation)
{
  const AccessPolicy policy = makeSubscribePolicy({"*"});

  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera//front/image"));
}

TEST(AccessPolicyTest, IgnoresConfiguredEntriesThatRosValidationRejects)
{
  const AccessPolicy policy = makeSubscribePolicy({"camera//front/*", "/camera/front/image"});

  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/front/stream"));
  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, ExactAndSubtreeRulesHaveDistinctMatchBoundaries)
{
  const AccessPolicy exact = makeSubscribePolicy({"/camera/front"});
  const AccessPolicy subtree = makeSubscribePolicy({"/camera/front/*"});

  EXPECT_TRUE(exact.allows(AccessOperation::Subscribe, "/camera/front"));
  EXPECT_FALSE(exact.allows(AccessOperation::Subscribe, "/camera/front/image"));

  EXPECT_FALSE(subtree.allows(AccessOperation::Subscribe, "/camera/front"));
  EXPECT_TRUE(subtree.allows(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, RootSubtreeWildcardAllowsAnyResourceExceptDeniedSubtrees)
{
  const AccessPolicy policy = makeSubscribePolicy({"/*"}, {"  private/*  "});

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/front/image"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/private/secret"));
}

}  // namespace livekit_ros2_bridge
