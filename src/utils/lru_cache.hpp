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

// Fixed-capacity, internally synchronized LRU cache. `get` hits and writes
// refresh recency; overflow evicts the least-recent entry. Zero capacity stays empty.
template <typename Key, typename Value, typename Hash = std::hash<Key>, typename KeyEqual = std::equal_to<Key>>
class LruCache
{
public:
  explicit LruCache(std::size_t capacity)
  : capacity_(capacity)
  {}

  LruCache(const LruCache &) = delete;
  LruCache & operator=(const LruCache &) = delete;

  std::optional<Value> get(const Key & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = index_.find(key);
    if (it == index_.end()) {
      return std::nullopt;
    }

    entries_.splice(entries_.end(), entries_, it->second);
    return it->second->value;
  }

  void set(Key key, Value value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = index_.find(key);
    if (it != index_.end()) {
      it->second->value = std::move(value);
      entries_.splice(entries_.end(), entries_, it->second);
      return;
    }

    entries_.push_back(Entry{std::move(key), std::move(value)});
    auto entry = std::prev(entries_.end());
    index_.emplace(entry->key, entry);
    if (index_.size() > capacity_) {
      index_.erase(entries_.front().key);
      entries_.pop_front();
    }
  }

private:
  struct Entry
  {
    Key key;
    Value value;
  };

  using Entries = std::list<Entry>;
  using Index = std::unordered_map<Key, typename Entries::iterator, Hash, KeyEqual>;

  std::size_t capacity_;
  std::mutex mutex_;
  // Front is least-recent, back is most-recent. The index stores list iterators;
  // same-list splice moves entries without invalidating them.
  Entries entries_;
  Index index_;
};

}  // namespace livekit_ros2_bridge
