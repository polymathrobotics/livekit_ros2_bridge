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
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "payloads/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

using Json = nlohmann::json;

namespace
{

constexpr char kInvalidLimitMessage[] = "limit must be a positive integer";

class ResourceListInvalidArgument final : public std::invalid_argument
{
public:
  ResourceListInvalidArgument(std::string_view field_name, const char * message)
  : std::invalid_argument(message)
  , field_name_(field_name)
  {}

  std::string_view fieldName() const noexcept
  {
    return field_name_;
  }

private:
  std::string_view field_name_;
};

[[noreturn]] void throwInvalidRequestField(std::string_view field_name, const char * message)
{
  throw ResourceListInvalidArgument(field_name, message);
}

}  // namespace

namespace resource_list_payloads
{

ResourceListRequest parse(const std::string & request_payload)
{
  Json request_body;
  try {
    request_body =
      parseJsonObject(request_payload, "Invalid JSON in list request", "List request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ResourceListInvalidArgument("payload", exc.what());
  }

  ResourceListRequest request;
  // Normalize blank and null queries to "no filter".
  try {
    request.query = parseOptionalNonEmptyTrimmedStringField(request_body, "query", "query must be a string", true);
  } catch (const std::invalid_argument & exc) {
    throw ResourceListInvalidArgument("query", exc.what());
  }

  const auto limit_it = request_body.find("limit");
  if (limit_it != request_body.end() && !limit_it->is_null()) {
    if (!limit_it->is_number_integer()) {
      throwInvalidRequestField("limit", kInvalidLimitMessage);
    }

    // Parse into a signed type first so negative JSON integers are rejected before converting to
    // the unsigned storage used by `ResourceListRequest::limit`.
    const auto limit = limit_it->get<std::int64_t>();
    if (limit <= 0) {
      throwInvalidRequestField("limit", kInvalidLimitMessage);
    }

    request.limit = static_cast<std::size_t>(limit);
  }

  return request;
}

std::optional<std::string_view> invalidRequestField(const std::exception & exc)
{
  if (const auto * resource_list_error = dynamic_cast<const ResourceListInvalidArgument *>(&exc)) {
    return resource_list_error->fieldName();
  }

  return std::nullopt;
}

std::string serializeServiceList(const std::vector<ResourceListEntry> & entries)
{
  Json services = Json::array();
  for (const auto & entry : entries) {
    services.push_back({{"name", entry.name}, {"interface_type", entry.interface_type}});
  }

  return Json{{"services", std::move(services)}}.dump();
}

std::string serializeTopicList(const std::vector<ResourceListEntry> & entries)
{
  Json topics = Json::array();
  for (const auto & entry : entries) {
    topics.push_back({{"name", entry.name}, {"interface_type", entry.interface_type}});
  }

  return Json{{"topics", std::move(topics)}}.dump();
}

}  // namespace resource_list_payloads

}  // namespace livekit_ros2_bridge
