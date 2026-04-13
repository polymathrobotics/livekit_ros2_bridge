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
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace livekit_ros2_bridge
{

/// Parsed form of a resource-list request. Unknown JSON fields are ignored.
struct ResourceListRequest
{
  /// Optional trimmed query string. Missing, null, or blank values are treated as absent.
  std::optional<std::string> query;
  /// Optional positive integer result cap. Missing or null means no explicit limit.
  std::optional<std::size_t> limit;
};

/// The wire format carries one `interface_type`, so callers must resolve or drop multi-type ROS
/// graph entries before serializing.
struct ResourceListEntry
{
  std::string name;
  std::string interface_type;
};

namespace resource_list_payloads
{

/// Parse a resource-list request object.
/// Throws `std::invalid_argument` when the payload is not a JSON object or `query` / `limit`
/// violate the RPC contract. Missing, null, and blank queries normalize to "no filter".
ResourceListRequest parse(const std::string & request_payload);

/// Return the request boundary associated with a resource-list validation error when available.
/// This lets higher layers add a precise `request_field` log field without parsing free-form
/// human-readable exception text.
std::optional<std::string_view> invalidRequestField(const std::exception & exc);

/// Serialize services as `{ "services": [{ "name", "interface_type" }, ...] }` in caller order.
/// Callers must pre-filter results, apply any limit, and collapse multi-type ROS graph entries to
/// the single `interface_type` wire shape before calling this helper.
std::string serializeServiceList(const std::vector<ResourceListEntry> & entries);

/// Serialize topics as `{ "topics": [{ "name", "interface_type" }, ...] }` in caller order.
/// Callers must pre-filter results, apply any limit, and collapse multi-type ROS graph entries to
/// the single `interface_type` wire shape before calling this helper.
std::string serializeTopicList(const std::vector<ResourceListEntry> & entries);

}  // namespace resource_list_payloads

}  // namespace livekit_ros2_bridge
