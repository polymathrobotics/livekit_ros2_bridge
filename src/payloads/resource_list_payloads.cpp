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

#include "payloads/resource_list_payloads.hpp"

#include <cstdint>

#include "nlohmann/json.hpp"
#include "utils/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using Json = nlohmann::json;

std::string serializeEntries(const char * key, const std::vector<ResourceListEntry> & entries)
{
  Json json_entries = Json::array();
  for (const auto & entry : entries) {
    json_entries.push_back({{"name", entry.name}, {"interface_type", entry.interface_type}});
  }

  const Json body = {{key, json_entries}};
  return body.dump();
}

}  // namespace

ResourceListRequest parseResourceListRequest(const std::string & payload)
{
  const Json json = parseJsonObject(payload, "Invalid JSON in list request", "List request must be a JSON object");

  ResourceListRequest request;

  request.query = parseOptionalNonEmptyTrimmedStringField(json, "query", "query must be a string", true);

  const auto limit_it = json.find("limit");
  const bool has_limit = limit_it != json.end() && !limit_it->is_null();
  if (has_limit) {
    if (!limit_it->is_number_integer()) {
      throw std::invalid_argument("limit must be a positive integer");
    }

    const auto limit = limit_it->get<std::int64_t>();
    if (limit <= 0) {
      throw std::invalid_argument("limit must be a positive integer");
    }

    request.limit = static_cast<std::size_t>(limit);
  }

  return request;
}

std::string serializeServiceListResponse(const std::vector<ResourceListEntry> & services)
{
  return serializeEntries("services", services);
}

std::string serializeTopicListResponse(const std::vector<ResourceListEntry> & topics)
{
  return serializeEntries("topics", topics);
}

}  // namespace livekit_ros2_bridge
