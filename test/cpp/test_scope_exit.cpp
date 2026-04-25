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

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "utils/scope_exit.hpp"

namespace livekit_ros2_bridge
{

TEST(ScopeExitTest, InvokesCallbackDuringExceptionUnwinding)
{
  int calls = 0;

  try {
    ScopeExit guard([&calls]() { ++calls; });
    throw std::runtime_error("boom");
  } catch (const std::runtime_error &) {}

  EXPECT_EQ(calls, 1);
}

TEST(ScopeExitTest, InvokesCallbacksOnNormalScopeExitInReverseConstructionOrder)
{
  std::vector<int> order;

  {
    ScopeExit first([&order]() { order.push_back(1); });
    ScopeExit second([&order]() { order.push_back(2); });
  }

  EXPECT_EQ((std::vector<int>{2, 1}), order);
}

TEST(ScopeExitTest, TerminatesWhenCallbackThrowsDuringDestruction)
{
  EXPECT_DEATH({ ScopeExit guard([]() { throw std::runtime_error("callback boom"); }); }, "callback boom");
}

}  // namespace livekit_ros2_bridge
