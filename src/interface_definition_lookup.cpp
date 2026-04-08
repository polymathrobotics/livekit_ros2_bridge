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

#include "interface_definition_lookup.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kSchemaEncodingRos2Msg[] = "ros2msg";

// ROS 2 primitive types that do not need dependency resolution.
const std::set<std::string> kPrimitiveTypes = {
  "bool",
  "byte",
  "char",
  "float32",
  "float64",
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
  "string",
  "wstring",
  "boolean",
  "octet",
};

struct ParsedInterfaceType
{
  std::string package;
  std::string kind;
  std::string name;
};

/// Parse "sensor_msgs/msg/BatteryState" into {package, kind, name}.
ParsedInterfaceType parseInterfaceType(const std::string & interface_type)
{
  // Expected: "package/kind/Name" where kind is msg, srv, or action
  const auto first_slash = interface_type.find('/');
  if (first_slash == std::string::npos) {
    throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': expected package/kind/Name");
  }
  const auto second_slash = interface_type.find('/', first_slash + 1);
  if (second_slash == std::string::npos) {
    throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': expected package/kind/Name");
  }

  ParsedInterfaceType result;
  result.package = interface_type.substr(0, first_slash);
  result.kind = interface_type.substr(first_slash + 1, second_slash - first_slash - 1);
  result.name = interface_type.substr(second_slash + 1);

  if (result.package.empty() || result.kind.empty() || result.name.empty()) {
    throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': empty component");
  }

  return result;
}

std::string readInterfaceDefinitionFile(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open schema file: " + path);
  }
  std::ostringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

std::string resolveInterfaceDefinitionPath(const ParsedInterfaceType & parsed)
{
  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory(parsed.package);
  } catch (const std::exception &) {
    throw std::runtime_error("Package '" + parsed.package + "' not found in ament index");
  }
  return share_dir + "/" + parsed.kind + "/" + parsed.name + "." + parsed.kind;
}

/// Extract the base type from a field type string, stripping array suffixes.
/// "float32[]" → "float32", "uint8[16]" → "uint8", "std_msgs/Header" → "std_msgs/Header"
std::string stripArraySuffix(const std::string & type_token)
{
  const auto bracket = type_token.find('[');
  if (bracket != std::string::npos) {
    return type_token.substr(0, bracket);
  }
  return type_token;
}

/// Normalize a type reference found in a .msg file to a fully-qualified type.
/// "std_msgs/Header" → "std_msgs/msg/Header" (short form)
/// "std_msgs/msg/Header" → "std_msgs/msg/Header" (already qualified)
std::string qualifyTypeReference(const std::string & type_ref, const std::string & context_subfolder)
{
  const auto first_slash = type_ref.find('/');
  if (first_slash == std::string::npos) {
    return type_ref;
  }
  const auto second_slash = type_ref.find('/', first_slash + 1);
  if (second_slash != std::string::npos) {
    return type_ref;
  }
  // Short form: "std_msgs/Header" → "std_msgs/msg/Header"
  const std::string package = type_ref.substr(0, first_slash);
  const std::string name = type_ref.substr(first_slash + 1);
  return package + "/" + context_subfolder + "/" + name;
}

/// Scan a .msg or .srv file for complex type references and return their fully-qualified names.
/// Field types are always messages, so short-form references like "std_msgs/Header" are
/// qualified as "std_msgs/msg/Header" regardless of whether the containing file is msg or srv.
std::vector<std::string> extractTypeReferences(const std::string & schema_text)
{
  std::vector<std::string> refs;
  std::istringstream stream(schema_text);
  std::string line;

  while (std::getline(stream, line)) {
    // Skip empty lines and comments
    const auto first_non_space = line.find_first_not_of(" \t");
    if (first_non_space == std::string::npos || line[first_non_space] == '#') {
      continue;
    }

    // Skip .srv separator lines
    if (line.compare(first_non_space, 3, "---") == 0) {
      continue;
    }

    // Skip constant definitions (lines containing '=')
    if (line.find('=') != std::string::npos) {
      continue;
    }

    // Extract the first token (the type)
    std::istringstream line_stream(line.substr(first_non_space));
    std::string type_token;
    line_stream >> type_token;
    if (type_token.empty()) {
      continue;
    }

    const std::string base_type = stripArraySuffix(type_token);

    // Skip primitives
    if (kPrimitiveTypes.count(base_type) > 0) {
      continue;
    }

    // Must contain a '/' to be a package-qualified type reference
    if (base_type.find('/') == std::string::npos) {
      continue;
    }

    refs.push_back(qualifyTypeReference(base_type, "msg"));
  }

  return refs;
}

void collectDependencies(
  const std::string & interface_type, std::set<std::string> & visited, std::vector<InterfaceDefinition> & dependencies)
{
  if (visited.count(interface_type) > 0) {
    return;
  }
  visited.insert(interface_type);

  const auto parsed = parseInterfaceType(interface_type);
  const std::string path = resolveInterfaceDefinitionPath(parsed);
  const std::string definition = readInterfaceDefinitionFile(path);

  dependencies.push_back({interface_type, kSchemaEncodingRos2Msg, definition});

  for (const auto & ref : extractTypeReferences(definition)) {
    collectDependencies(ref, visited, dependencies);
  }
}

}  // namespace

InterfaceDefinitions lookupInterfaceDefinitions(const std::string & interface_type)
{
  std::set<std::string> visited;
  std::vector<InterfaceDefinition> entries;
  collectDependencies(interface_type, visited, entries);

  InterfaceDefinitions result;
  result.requested = std::move(entries.front());
  result.dependencies.assign(std::make_move_iterator(entries.begin() + 1), std::make_move_iterator(entries.end()));
  return result;
}

}  // namespace livekit_ros2_bridge
