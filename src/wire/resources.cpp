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

#include "wire/resources.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "wire/detail/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

using Json = nlohmann::json;

namespace
{

constexpr char kInvalidLimitMessage[] = "limit must be a positive integer";

class InvalidRequest final : public std::invalid_argument
{
public:
  InvalidRequest(std::string_view field, const char * message)
  : std::invalid_argument(message)
  , field_(field)
  {}

  std::string_view field() const noexcept
  {
    return field_;
  }

private:
  std::string_view field_;
};

std::optional<std::size_t> parseLimit(const Json & body)
{
  const auto field = body.find("limit");
  if (field == body.end() || field->is_null()) {
    return std::nullopt;
  }

  if (!field->is_number_integer()) {
    throw InvalidRequest("limit", kInvalidLimitMessage);
  }

  // Parse into a signed type first so negative JSON integers are rejected before converting to
  // the unsigned storage used by `ResourceListRequest::limit`.
  const auto limit = field->get<std::int64_t>();
  if (limit <= 0) {
    throw InvalidRequest("limit", kInvalidLimitMessage);
  }

  return static_cast<std::size_t>(limit);
}

std::optional<std::string> parseQuery(const Json & body)
{
  // Normalize blank and null queries to "no filter".
  try {
    return wire::detail::parseOptionalNonEmptyTrimmedStringField(body, "query", "query must be a string", true);
  } catch (const std::invalid_argument & exc) {
    throw InvalidRequest("query", exc.what());
  }
}

std::string serializeEntries(std::string_view key, const std::vector<ResourceEntry> & entries)
{
  Json resources = Json::array();
  for (const auto & entry : entries) {
    resources.push_back({{"name", entry.name}, {"interface_type", entry.interface_type}});
  }

  return Json{{key, std::move(resources)}}.dump();
}

}  // namespace

namespace wire::resources
{

ResourceListRequest parse(const std::string & payload)
{
  Json body;
  try {
    body = wire::detail::parseJsonObject(payload, "Invalid JSON in list request", "List request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw InvalidRequest("payload", exc.what());
  }

  return {parseQuery(body), parseLimit(body)};
}

std::optional<std::string_view> invalidRequestField(const std::exception & exc)
{
  if (const auto * error = dynamic_cast<const InvalidRequest *>(&exc)) {
    return error->field();
  }

  return std::nullopt;
}

std::string serializeServices(const std::vector<ResourceEntry> & entries)
{
  return serializeEntries("services", entries);
}

std::string serializeTopics(const std::vector<ResourceEntry> & entries)
{
  return serializeEntries("topics", entries);
}

}  // namespace wire::resources

}  // namespace livekit_ros2_bridge
