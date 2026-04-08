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
#include <optional>
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

struct ResourceListRequest
{
  std::optional<std::string> query;
  std::optional<std::size_t> limit;
};

struct ResourceListEntry
{
  std::string name;
  std::string interface_type;
};

ResourceListRequest parseResourceListRequest(const std::string & payload);

std::string serializeServiceListResponse(const std::vector<ResourceListEntry> & services);

std::string serializeTopicListResponse(const std::vector<ResourceListEntry> & topics);

}  // namespace livekit_ros2_bridge
