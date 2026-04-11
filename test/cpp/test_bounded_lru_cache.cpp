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

#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "utils/bounded_lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::string expectRuntimeErrorMessage(const std::function<void()> & action)
{
  try {
    action();
    ADD_FAILURE() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    return exc.what();
  } catch (...) {
    ADD_FAILURE() << "Expected std::runtime_error";
  }
  return "";
}

TEST(BoundedLruCacheTest, StoresAndReturnsGenericValues)
{
  BoundedLruCache<std::string, int> cache(2U);
  cache.insertOrAssign("alpha", 7);

  ASSERT_EQ(cache.size(), 1U);
  ASSERT_TRUE(cache.get("alpha").has_value());
  EXPECT_EQ(*cache.get("alpha"), 7);
  EXPECT_FALSE(cache.get("missing").has_value());
}

TEST(BoundedLruCacheTest, TouchingHitPreservesItDuringLruEviction)
{
  BoundedLruCache<std::string, int> cache(2U);
  cache.insertOrAssign("alpha", 1);
  cache.insertOrAssign("beta", 2);

  ASSERT_TRUE(cache.get("alpha").has_value());
  cache.insertOrAssign("gamma", 3);

  EXPECT_EQ(cache.size(), 2U);
  EXPECT_TRUE(cache.get("alpha").has_value());
  EXPECT_FALSE(cache.get("beta").has_value());
  ASSERT_TRUE(cache.get("gamma").has_value());
  EXPECT_EQ(*cache.get("gamma"), 3);
}

TEST(BoundedLruCacheTest, PeekDoesNotRefreshRecencyButTouchDoes)
{
  BoundedLruCache<std::string, int> peek_cache(2U);
  peek_cache.insertOrAssign("alpha", 1);
  peek_cache.insertOrAssign("beta", 2);

  ASSERT_TRUE(peek_cache.peek("alpha").has_value());
  peek_cache.insertOrAssign("gamma", 3);
  EXPECT_FALSE(peek_cache.peek("alpha").has_value());

  BoundedLruCache<std::string, int> touch_cache(2U);
  touch_cache.insertOrAssign("alpha", 1);
  touch_cache.insertOrAssign("beta", 2);
  ASSERT_TRUE(touch_cache.touch("alpha"));
  touch_cache.insertOrAssign("gamma", 3);
  EXPECT_TRUE(touch_cache.peek("alpha").has_value());
  EXPECT_FALSE(touch_cache.peek("beta").has_value());
}

TEST(BoundedLruCacheTest, InsertReturnsEvictedEntryWhenCapacityExceeded)
{
  BoundedLruCache<std::string, int> cache(1U);
  EXPECT_FALSE(cache.insertOrAssign("alpha", 1).has_value());

  const auto evicted = cache.insertOrAssign("beta", 2);
  ASSERT_TRUE(evicted.has_value());
  EXPECT_EQ(evicted->key, "alpha");
  EXPECT_EQ(evicted->value, 1);
}

TEST(BoundedLruCacheTest, SupportsFailureCacheUsageWithExceptionPtrs)
{
  BoundedLruCache<std::string, std::exception_ptr> cache(2U);
  cache.insertOrAssign("alpha", std::make_exception_ptr(std::runtime_error("bad alpha")));

  const auto failure = cache.get("alpha");
  ASSERT_TRUE(failure.has_value());
  EXPECT_EQ(expectRuntimeErrorMessage([&failure]() { std::rethrow_exception(*failure); }), "bad alpha");
}

}  // namespace

}  // namespace livekit_ros2_bridge
