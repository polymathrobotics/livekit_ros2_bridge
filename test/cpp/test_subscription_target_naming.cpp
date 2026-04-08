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

#include "gtest/gtest.h"
#include "utils/subscription_target_naming.hpp"

namespace livekit_ros2_bridge
{

TEST(SubscriptionTargetNamingTest, SubscriptionTargetKeyIncludesKindAndResource)
{
  const SubscriptionTarget topic_target{SubscriptionTargetKind::Topic, "/camera"};
  const SubscriptionTarget external_target{SubscriptionTargetKind::External, "/camera"};

  const std::string topic_key = makeSubscriptionTargetKey(topic_target);
  const std::string external_key = makeSubscriptionTargetKey(external_target);

  EXPECT_EQ(topic_key, "topic:/camera");
  EXPECT_EQ(external_key, "external:/camera");
  EXPECT_NE(topic_key, external_key);
}

}  // namespace livekit_ros2_bridge
