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

#include "payloads/interface_payloads.hpp"

#include <stdexcept>

#include "nlohmann/json.hpp"
#include "utils/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using Json = nlohmann::json;

}  // namespace

std::vector<std::string> parseRequestedInterfaceTypes(const std::string & payload)
{
  const Json json =
    parseJsonObject(payload, "Invalid JSON in interfaces get request", "Interfaces get request must be a JSON object");

  const auto types_it = json.find("interface_types");
  if (types_it == json.end() || !types_it->is_array()) {
    throw std::invalid_argument("interface_types must be an array");
  }

  std::vector<std::string> types;
  for (const auto & element : *types_it) {
    if (!element.is_string()) {
      throw std::invalid_argument("interface_types entries must be strings");
    }
    auto trimmed = element.get<std::string>();
    const auto start = trimmed.find_first_not_of(" \t\n\r");
    const auto end = trimmed.find_last_not_of(" \t\n\r");
    if (start == std::string::npos) {
      throw std::invalid_argument("interface_types entries must not be empty");
    }
    trimmed = trimmed.substr(start, end - start + 1);
    if (trimmed.empty()) {
      throw std::invalid_argument("interface_types entries must not be empty");
    }
    types.push_back(std::move(trimmed));
  }

  if (types.empty()) {
    throw std::invalid_argument("interface_types must not be empty");
  }

  return types;
}

std::string serializeInterfacesResponse(const std::vector<InterfaceDefinition> & interfaces)
{
  Json entries = Json::array();
  for (const auto & iface : interfaces) {
    entries.push_back({
      {"interface_type", iface.interface_type},
      {"schema_encoding", iface.schema_encoding},
      {"definition", iface.definition},
    });
  }

  const Json body = {{"interfaces", entries}};
  return body.dump();
}

}  // namespace livekit_ros2_bridge
