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

#include "protocol/resources_json.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"

namespace livekit_ros2_bridge::protocol::resources
{

using Json = nlohmann::json;

ResourceListRequest parse(const std::string & payload)
{
  Json body;
  try {
    body = detail::parseObject(payload, "Invalid JSON in list request", "List request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("payload", exc.what());
  }

  std::optional<std::string> query;
  try {
    query = detail::optionalString(body, "query", "query must be a string", /*null_is_absent=*/true);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("query", exc.what());
  }

  std::optional<std::size_t> limit;
  const auto field = body.find("limit");
  if (field != body.end() && !field->is_null()) {
    constexpr char reason[] = "limit must be a positive integer";
    if (!field->is_number_integer()) {
      throw ValidationError("limit", reason);
    }

    // Read signed so negative JSON integers fail before size_t conversion.
    const auto value = field->get<std::int64_t>();
    if (value <= 0) {
      throw ValidationError("limit", reason);
    }

    limit = static_cast<std::size_t>(value);
  }

  return {std::move(query), limit};
}

std::string serializeServices(const ResourceTypesByName & resources)
{
  Json entries = Json::array();
  for (const auto & [service, types] : resources) {
    entries.push_back({{"service", service}, {"interface_type", types.front()}});
  }

  return Json{{"services", std::move(entries)}}.dump();
}

std::string serializeTopics(const ResourceTypesByName & resources)
{
  Json entries = Json::array();
  for (const auto & [topic, types] : resources) {
    entries.push_back({{"topic", topic}, {"interface_type", types.front()}});
  }

  return Json{{"topics", std::move(entries)}}.dump();
}

}  // namespace livekit_ros2_bridge::protocol::resources
