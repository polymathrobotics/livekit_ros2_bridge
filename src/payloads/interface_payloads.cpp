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

#include <utility>

#include "nlohmann/json.hpp"
#include "payloads/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

using Json = nlohmann::json;

namespace interface_payloads
{

std::vector<std::string> parse(const std::string & payload)
{
  return parseRequiredNonEmptyTrimmedStringArrayField(
    parseJsonObject(payload, "Invalid JSON in interfaces get request", "Interfaces get request must be a JSON object"),
    "interface_types",
    "interface_types must be an array",
    "interface_types entries must be strings",
    "interface_types entries must not be empty",
    "interface_types must not be empty");
}

std::string serialize(const std::vector<InterfaceDefinition> & definitions)
{
  Json::array_t entries;
  entries.reserve(definitions.size());
  for (const auto & definition : definitions) {
    entries.push_back(
      Json{
        {"interface_type", definition.interface_type},
        {"format", definition.format},
        {"definition", definition.definition},
      });
  }

  return Json{{"interfaces", std::move(entries)}}.dump();
}

}  // namespace interface_payloads

}  // namespace livekit_ros2_bridge
