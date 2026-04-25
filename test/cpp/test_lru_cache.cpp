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

#include <atomic>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "utils/lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

TEST(LruCacheTest, GetReturnsStoredValueRefreshesRecencyAndMissesMissingKeys)
{
  LruCache<std::string, int> cache(2U);
  cache.insertOrAssign("alpha", 1);
  cache.insertOrAssign("beta", 2);

  EXPECT_EQ(cache.get("alpha"), std::optional<int>{1});
  cache.insertOrAssign("gamma", 3);

  EXPECT_EQ(cache.get("alpha"), std::optional<int>{1});
  EXPECT_FALSE(cache.get("beta").has_value());
  EXPECT_EQ(cache.get("gamma"), std::optional<int>{3});
  EXPECT_FALSE(cache.get("missing").has_value());
}

TEST(LruCacheTest, InsertOrAssignExistingKeyUpdatesValueAndRefreshesRecency)
{
  LruCache<std::string, int> cache(2U);
  cache.insertOrAssign("alpha", 1);
  cache.insertOrAssign("beta", 2);

  cache.insertOrAssign("alpha", 10);

  cache.insertOrAssign("gamma", 3);
  EXPECT_EQ(cache.get("alpha"), std::optional<int>{10});
  EXPECT_FALSE(cache.get("beta").has_value());
}

TEST(LruCacheTest, InsertOrAssignEvictsLeastRecentEntry)
{
  LruCache<std::string, int> cache(1U);
  cache.insertOrAssign("alpha", 1);

  cache.insertOrAssign("beta", 2);

  EXPECT_FALSE(cache.get("alpha").has_value());
  EXPECT_EQ(cache.get("beta"), std::optional<int>{2});
}

TEST(LruCacheTest, SupportsFailureCacheUsageWithExceptionPtrs)
{
  LruCache<std::string, std::exception_ptr> cache(2U);
  cache.insertOrAssign("alpha", std::make_exception_ptr(std::runtime_error("bad alpha")));

  const auto failure = cache.get("alpha");
  ASSERT_TRUE(failure.has_value());
  try {
    std::rethrow_exception(*failure);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), "bad alpha");
  }
}

TEST(LruCacheTest, ConcurrentAccessKeepsCacheUsable)
{
  constexpr std::size_t kCapacity = 8U;
  constexpr int kThreadCount = 4;
  constexpr int kIterationsPerThread = 200;

  LruCache<std::string, int> cache(kCapacity);
  std::atomic<int> ready_count{0};
  std::atomic<bool> start{false};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (int thread_index = 0; thread_index < kThreadCount; ++thread_index) {
    threads.emplace_back([&, thread_index]() {
      ready_count.fetch_add(1, std::memory_order_relaxed);
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      for (int iteration = 0; iteration < kIterationsPerThread; ++iteration) {
        const std::string key = "key_" + std::to_string((thread_index + iteration) % static_cast<int>(kCapacity * 2U));

        if (iteration % 3 == 0) {
          cache.insertOrAssign(key, thread_index * 1000 + iteration);
        } else {
          (void)cache.get(key);
        }
      }
    });
  }

  while (ready_count.load(std::memory_order_acquire) < kThreadCount) {
    std::this_thread::yield();
  }
  start.store(true, std::memory_order_release);

  for (auto & thread : threads) {
    thread.join();
  }

  cache.insertOrAssign("sentinel", 42);
  EXPECT_EQ(cache.get("sentinel"), std::optional<int>{42});
}

}  // namespace

}  // namespace livekit_ros2_bridge
