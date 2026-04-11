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

#include <cstddef>
#include <functional>
#include <iterator>
#include <list>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>

namespace livekit_ros2_bridge
{

// Small thread-safe LRU cache for copyable values. Reads touch recency, and
// inserts evict the least-recently used entry once the fixed capacity is exceeded.
template <typename Key, typename Value, typename Hash = std::hash<Key>, typename KeyEqual = std::equal_to<Key>>
class BoundedLruCache
{
public:
  explicit BoundedLruCache(std::size_t capacity)
  : capacity_(capacity)
  {}

  BoundedLruCache(const BoundedLruCache &) = delete;
  BoundedLruCache & operator=(const BoundedLruCache &) = delete;

  std::optional<Value> get(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      return std::nullopt;
    }

    lru_entries_.splice(lru_entries_.end(), lru_entries_, it->second);
    return it->second->value;
  }

  void insertOrAssign(Key key, Value value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it != entries_.end()) {
      it->second->value = std::move(value);
      lru_entries_.splice(lru_entries_.end(), lru_entries_, it->second);
      return;
    }

    lru_entries_.push_back(CacheEntry{std::move(key), std::move(value)});
    auto cache_entry_it = std::prev(lru_entries_.end());
    entries_.emplace(cache_entry_it->key, cache_entry_it);
    evictIfNeeded();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    lru_entries_.clear();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

private:
  struct CacheEntry
  {
    Key key;
    Value value;
  };

  using CacheEntries = std::list<CacheEntry>;
  using CacheIndex = std::unordered_map<Key, typename CacheEntries::iterator, Hash, KeyEqual>;

  void evictIfNeeded()
  {
    while (entries_.size() > capacity_) {
      entries_.erase(lru_entries_.front().key);
      lru_entries_.pop_front();
    }
  }

  std::size_t capacity_ = 0U;
  mutable std::mutex mutex_;
  CacheEntries lru_entries_;
  CacheIndex entries_;
};

}  // namespace livekit_ros2_bridge
