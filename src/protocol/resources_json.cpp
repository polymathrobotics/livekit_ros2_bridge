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

namespace
{

constexpr char kInvalidLimitMessage[] = "limit must be a positive integer";

template <typename AddEntryT>
void appendSingleInterfaceTypeResources(
  Json & entries, const ResourceNamesAndTypes & resources_by_name, AddEntryT add_entry)
{
  for (const auto & [name, types] : resources_by_name) {
    if (types.size() != 1U) {
      continue;
    }
    add_entry(entries, name, types.front());
  }
}

std::optional<std::size_t> parseLimit(const Json & body)
{
  const auto field = body.find("limit");
  if (field == body.end() || field->is_null()) {
    return std::nullopt;
  }

  if (!field->is_number_integer()) {
    throw ValidationError("limit", kInvalidLimitMessage);
  }

  // Parse into a signed type first so negative JSON integers are rejected before converting to
  // the unsigned storage used by `ResourceListRequest::limit`.
  const auto limit = field->get<std::int64_t>();
  if (limit <= 0) {
    throw ValidationError("limit", kInvalidLimitMessage);
  }

  return static_cast<std::size_t>(limit);
}

}  // namespace

ResourceListRequest parseRequest(const std::string & payload)
{
  Json body;
  try {
    body = detail::parseObject(payload, "Invalid JSON in list request", "List request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("payload", exc.what());
  }

  std::optional<std::string> query;
  try {
    // Normalize blank and null queries to "no filter".
    query = detail::optionalTrimmedStringField(body, "query", "query must be a string", true);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("query", exc.what());
  }

  return {std::move(query), parseLimit(body)};
}

std::string serializeServices(const ResourceNamesAndTypes & resources_by_name)
{
  Json entries = Json::array();
  appendSingleInterfaceTypeResources(
    entries, resources_by_name, [](Json & output, const std::string & service, const std::string & interface_type) {
      output.push_back({{"service", service}, {"interface_type", interface_type}});
    });

  return Json{{"services", std::move(entries)}}.dump();
}

std::string serializeTopics(const ResourceNamesAndTypes & resources_by_name)
{
  Json entries = Json::array();
  appendSingleInterfaceTypeResources(
    entries, resources_by_name, [](Json & output, const std::string & topic, const std::string & interface_type) {
      output.push_back({{"topic", topic}, {"interface_type", interface_type}});
    });

  return Json{{"topics", std::move(entries)}}.dump();
}

}  // namespace livekit_ros2_bridge::protocol::resources
