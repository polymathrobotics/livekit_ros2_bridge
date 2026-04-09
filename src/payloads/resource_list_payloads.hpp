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

/// Parsed form of a topics/services list request. Unknown JSON fields are ignored.
struct ResourceListRequest
{
  /// Optional trimmed query string. Missing, null, or blank values are treated as absent.
  std::optional<std::string> query;
  /// Optional positive integer result cap. Missing or null means no explicit limit.
  std::optional<std::size_t> limit;
};

/// A single topics/services list response entry.
struct ResourceListEntry
{
  std::string name;
  std::string interface_type;
};

/// Parse a JSON object request with optional `query` and `limit` fields.
ResourceListRequest parseResourceListRequest(const std::string & payload);

/// Serialize a response body as `{ "services": [{ "name", "interface_type" }, ...] }`.
std::string serializeServiceListResponse(const std::vector<ResourceListEntry> & services);

/// Serialize a response body as `{ "topics": [{ "name", "interface_type" }, ...] }`.
std::string serializeTopicListResponse(const std::vector<ResourceListEntry> & topics);

}  // namespace livekit_ros2_bridge
