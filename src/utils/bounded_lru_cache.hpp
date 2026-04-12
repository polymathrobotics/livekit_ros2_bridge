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
  struct EvictedEntry
  {
    Key key;
    Value value;
  };

  explicit BoundedLruCache(std::size_t capacity)
  : capacity_(capacity)
  {}

  BoundedLruCache(const BoundedLruCache &) = delete;
  BoundedLruCache & operator=(const BoundedLruCache &) = delete;

  std::optional<Value> get(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entry_index_.find(key);
    if (it == entry_index_.end()) {
      return std::nullopt;
    }

    entries_by_recency_.splice(entries_by_recency_.end(), entries_by_recency_, it->second);
    return it->second->value;
  }

  std::optional<Value> peek(const Key & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entry_index_.find(key);
    if (it == entry_index_.end()) {
      return std::nullopt;
    }

    return it->second->value;
  }

  bool touch(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entry_index_.find(key);
    if (it == entry_index_.end()) {
      return false;
    }

    entries_by_recency_.splice(entries_by_recency_.end(), entries_by_recency_, it->second);
    return true;
  }

  std::optional<EvictedEntry> insertOrAssign(Key key, Value value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entry_index_.find(key);
    if (it != entry_index_.end()) {
      it->second->value = std::move(value);
      entries_by_recency_.splice(entries_by_recency_.end(), entries_by_recency_, it->second);
      return std::nullopt;
    }

    entries_by_recency_.push_back(LruEntry{std::move(key), std::move(value)});
    auto lru_entry_it = std::prev(entries_by_recency_.end());
    entry_index_.emplace(lru_entry_it->key, lru_entry_it);
    return evictIfNeeded();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entry_index_.clear();
    entries_by_recency_.clear();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entry_index_.size();
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entry_index_.empty();
  }

private:
  struct LruEntry
  {
    Key key;
    Value value;
  };

  using LruEntries = std::list<LruEntry>;
  using EntryIndex = std::unordered_map<Key, typename LruEntries::iterator, Hash, KeyEqual>;

  std::optional<EvictedEntry> evictIfNeeded()
  {
    if (entry_index_.size() <= capacity_) {
      return std::nullopt;
    }

    EvictedEntry evicted{std::move(entries_by_recency_.front().key), std::move(entries_by_recency_.front().value)};
    entry_index_.erase(evicted.key);
    entries_by_recency_.pop_front();
    return evicted;
  }

  std::size_t capacity_ = 0U;
  mutable std::mutex mutex_;
  LruEntries entries_by_recency_;
  EntryIndex entry_index_;
};

}  // namespace livekit_ros2_bridge
