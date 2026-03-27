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

#include "gtest/gtest.h"
#include "livekit_ros2_bridge/access_policy.hpp"

namespace livekit_ros2_bridge
{

TEST(AccessPolicyTest, DeniesWhenAllowlistIsEmpty)
{
  const StaticAccessPolicy policy({}, {}, {}, {});

  EXPECT_FALSE(policy.authorize(AccessOperation::Subscribe, "/anything"));
}

TEST(AccessPolicyTest, AllowAllStillHonorsDenylist)
{
  const StaticAccessPolicy policy({"*"}, {"/blocked"}, {}, {});

  EXPECT_TRUE(policy.authorize(AccessOperation::Subscribe, "/ok"));
  EXPECT_FALSE(policy.authorize(AccessOperation::Subscribe, "/blocked"));
}

TEST(AccessPolicyTest, SubtreeEntriesMatchDescendantsRecursively)
{
  const StaticAccessPolicy policy({"/camera/*"}, {}, {}, {});

  EXPECT_FALSE(policy.authorize(AccessOperation::Subscribe, "/camera"));
  EXPECT_TRUE(policy.authorize(AccessOperation::Subscribe, "/camera/image"));
  EXPECT_TRUE(policy.authorize(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, DenyAllWildcardBlocksEverything)
{
  const StaticAccessPolicy policy({"*"}, {"*"}, {}, {});

  EXPECT_FALSE(policy.authorize(AccessOperation::Subscribe, "/camera"));
  EXPECT_FALSE(policy.authorize(AccessOperation::Subscribe, "/camera/front/image"));
}

TEST(AccessPolicyTest, ServiceDenylistWinsOverAllowlist)
{
  const StaticAccessPolicy policy({}, {}, {"/example/service"}, {"/example/service"});

  EXPECT_FALSE(policy.authorize(AccessOperation::CallService, "/example/service"));
}

}  // namespace livekit_ros2_bridge
