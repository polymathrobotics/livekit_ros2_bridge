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
#include <exception>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace livekit_ros2_bridge::ros_interfaces
{

class FailureCache
{
public:
  explicit FailureCache(std::size_t capacity)
  : capacity_(capacity)
  {}

  FailureCache(const FailureCache &) = delete;
  FailureCache & operator=(const FailureCache &) = delete;

  std::optional<std::exception_ptr> find(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = failures_.find(key);
    if (it == failures_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  void remember(std::string key, std::exception_ptr failure)
  {
    if (capacity_ == 0U) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = failures_.find(key);
    if (it != failures_.end()) {
      it->second = std::move(failure);
      return;
    }

    if (failures_.size() >= capacity_) {
      failures_.clear();
    }
    failures_.emplace(std::move(key), std::move(failure));
  }

private:
  std::size_t capacity_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::exception_ptr> failures_;
};

}  // namespace livekit_ros2_bridge::ros_interfaces
