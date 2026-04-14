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

#include "wire/interfaces.hpp"

#include <exception>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "nlohmann/json.hpp"
#include "wire/detail/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

using Json = nlohmann::json;

namespace wire::interfaces
{

namespace
{

class InvalidFieldArgument final : public std::invalid_argument
{
public:
  InvalidFieldArgument(std::string_view field, const char * message)
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

}  // namespace

std::vector<std::string> parse(const std::string & payload)
{
  Json body;
  try {
    body = wire::detail::parseJsonObject(
      payload, "Invalid JSON in interfaces get request", "Interfaces get request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("payload", exc.what());
  }

  try {
    return wire::detail::parseRequiredNonEmptyTrimmedStringArrayField(
      body,
      "interface_types",
      "interface_types must be an array",
      "interface_types entries must be strings",
      "interface_types entries must not be empty",
      "interface_types must not be empty");
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("interface_types", exc.what());
  }
}

std::optional<std::string_view> invalidRequestField(const std::exception & exc)
{
  const auto * error = dynamic_cast<const InvalidFieldArgument *>(&exc);
  if (error == nullptr) {
    return std::nullopt;
  }

  return error->field();
}

std::string serialize(const std::vector<InterfaceDefinition> & definitions)
{
  Json::array_t interfaces;
  interfaces.reserve(definitions.size());
  for (const auto & definition : definitions) {
    interfaces.push_back(
      Json{
        {"interface_type", definition.interface_type},
        {"format", definition.format},
        {"definition", definition.definition},
      });
  }

  return Json{{"interfaces", std::move(interfaces)}}.dump();
}

}  // namespace wire::interfaces

}  // namespace livekit_ros2_bridge
