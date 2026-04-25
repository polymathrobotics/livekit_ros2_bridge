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

#include "protocol/interfaces_json.hpp"

#include <stdexcept>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::protocol::interfaces
{

using Json = nlohmann::json;

namespace
{

constexpr char kPayloadField[] = "payload";
constexpr char kTypesField[] = "interface_types";
constexpr char kInterfacesField[] = "interfaces";
constexpr char kTypeField[] = "interface_type";
constexpr char kFormatField[] = "format";
constexpr char kDefinitionField[] = "definition";

}  // namespace

std::vector<std::string> parse(const std::string & payload)
{
  Json body;
  try {
    body = detail::parseObject(
      payload, "Invalid JSON in interface show request", "Interface show request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayloadField, exc.what());
  }

  try {
    const auto entries = body.find(kTypesField);
    if (entries == body.end() || !entries->is_array()) {
      throw std::invalid_argument("interface_types must be an array");
    }

    std::vector<std::string> types;
    types.reserve(entries->size());
    for (const auto & entry : *entries) {
      if (!entry.is_string()) {
        throw std::invalid_argument("interface_types entries must be strings");
      }

      const auto type = trim(entry.get_ref<const std::string &>());
      if (type.empty()) {
        throw std::invalid_argument("interface_types entries must not be empty");
      }

      types.push_back(type);
    }

    if (types.empty()) {
      throw std::invalid_argument("interface_types must not be empty");
    }

    return types;
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kTypesField, exc.what());
  }
}

std::string serialize(const std::vector<InterfaceDefinition> & definitions)
{
  Json::array_t entries;
  entries.reserve(definitions.size());
  for (const auto & definition : definitions) {
    entries.push_back(
      Json{
        {kTypeField, definition.type},
        {kFormatField, "ros2msg"},
        {kDefinitionField, definition.body},
      });
  }

  return Json{{kInterfacesField, std::move(entries)}}.dump();
}

}  // namespace livekit_ros2_bridge::protocol::interfaces
