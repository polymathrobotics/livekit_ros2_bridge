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

#include <vector>

#include "access_policy.hpp"
#include "gtest/gtest.h"

namespace livekit_ros2_bridge
{

namespace
{

AccessPolicy makeSubscribePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  return AccessPolicy({}, {}, allow, deny, {}, {});
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
  const AccessPolicy policy(
    {"/cmd/*"}, {"/cmd/blocked"}, {"/camera/image"}, {}, {"/example/service"}, {"/example/service"});

  EXPECT_TRUE(policy.allows(AccessOperation::Publish, "/cmd/allowed"));
  EXPECT_FALSE(policy.allows(AccessOperation::Publish, "/cmd/blocked"));

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/cmd/allowed"));

  EXPECT_FALSE(policy.allows(AccessOperation::CallService, "/example/service"));
}

TEST(AccessPolicyTest, AllowAllStillHonorsDenylistEntries)
{
  const AccessPolicy exact_deny_policy = makeSubscribePolicy({"  *  "}, {"  /blocked  "});
  const AccessPolicy wildcard_deny_policy = makeSubscribePolicy({"*"}, {"  *  "});

  EXPECT_TRUE(exact_deny_policy.allows(AccessOperation::Subscribe, "/ok"));
  EXPECT_FALSE(exact_deny_policy.allows(AccessOperation::Subscribe, "/blocked"));
  EXPECT_FALSE(wildcard_deny_policy.allows(AccessOperation::Subscribe, "/"));
  EXPECT_FALSE(wildcard_deny_policy.allows(AccessOperation::Subscribe, "/camera"));
  EXPECT_FALSE(wildcard_deny_policy.allows(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, TrimsAndNormalizesConfiguredEntries)
{
  const AccessPolicy policy = makeSubscribePolicy(
    {"   ", "  /camera/image  ", "  camera//front//*  "}, {"  blocked//tree//*  ", "  /camera/front/blocked  "});

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/front/stream"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/front/blocked"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/blocked/tree/leaf"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/other"));
}

TEST(AccessPolicyTest, IgnoresBlankConfiguredEntriesWithoutChangingPolicySemantics)
{
  const AccessPolicy policy = makeSubscribePolicy(
    {"   ", "\t", "/camera/image"}, {"   ", "\n", "  /camera/blocked  "});

  EXPECT_TRUE(policy.allows(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/blocked"));
  EXPECT_FALSE(policy.allows(AccessOperation::Subscribe, "/camera/other"));
}

}  // namespace livekit_ros2_bridge
