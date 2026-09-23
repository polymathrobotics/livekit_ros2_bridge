// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <unordered_set>

namespace livekit_ros2_bridge::utils
{

// Records `id` in `seen` and rejects duplicates and ids without generated
// parameter entries. Shared by runtime and media-kind config loaders so both
// report identical duplicate/missing messages.
template <typename EntryMap>
const typename EntryMap::mapped_type & requireUniqueEntry(
  std::unordered_set<std::string> & seen,
  const std::string & id,
  const EntryMap & entries,
  const char * duplicate_label,
  const char * missing_label)
{
  if (!seen.emplace(id).second) {
    throw std::runtime_error(std::string("duplicate ") + duplicate_label + " '" + id + "'");
  }

  const auto it = entries.find(id);
  if (it == entries.end()) {
    throw std::runtime_error(std::string(missing_label) + " '" + id + "' is missing generated parameters");
  }

  return it->second;
}

}  // namespace livekit_ros2_bridge::utils
