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

#include <exception>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_resource.hpp"
#include "utils/log_event.hpp"
#include "utils/lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kServiceDefinitionSeparator[] = "---";
constexpr char kRosidlInterfacesResourceType[] = "rosidl_interfaces";
constexpr std::size_t kInvalidTypeCacheCapacity = 256U;
const auto kLogger = rclcpp::get_logger("interface_definition_lookup");

using LookupFailureCache = LruCache<std::string, std::exception_ptr>;

LookupFailureCache & lookupFailureCache()
{
  static LookupFailureCache cache(kInvalidTypeCacheCapacity);
  return cache;
}

[[noreturn]] void throwInvalidInterfaceType(const std::string & interface_type, const char * reason)
{
  throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': " + reason);
}

struct TypeParts
{
  std::string package;
  std::string kind;
  std::string name;

  static TypeParts parse(const std::string & interface_type)
  {
    const auto first_slash = interface_type.find('/');
    if (first_slash == std::string::npos) {
      throwInvalidInterfaceType(interface_type, "expected package/kind/Name");
    }

    const auto second_slash = interface_type.find('/', first_slash + 1);
    if (second_slash == std::string::npos) {
      throwInvalidInterfaceType(interface_type, "expected package/kind/Name");
    }

    const std::string package = interface_type.substr(0, first_slash);
    const std::string kind = interface_type.substr(first_slash + 1, second_slash - first_slash - 1);
    const std::string name = interface_type.substr(second_slash + 1);
    if (package.empty() || kind.empty() || name.empty()) {
      throwInvalidInterfaceType(interface_type, "empty component");
    }
    if (name.find('/') != std::string::npos) {
      throwInvalidInterfaceType(interface_type, "expected package/kind/Name");
    }
    if (kind != "msg" && kind != "srv" && kind != "action") {
      throwInvalidInterfaceType(interface_type, "kind must be msg, srv, or action");
    }

    return {package, kind, name};
  }
};

std::filesystem::path resolveInterfaceDefinitionPath(const TypeParts & parts)
{
  std::string resource_content;
  std::string package_prefix;
  if (!ament_index_cpp::get_resource(kRosidlInterfacesResourceType, parts.package, resource_content, &package_prefix)) {
    throw std::runtime_error("Package '" + parts.package + "' not found in ament index");
  }

  const std::string requested_relative_path = parts.kind + "/" + parts.name + "." + parts.kind;
  std::istringstream lines(resource_content);
  std::string registered_relative_path;
  while (std::getline(lines, registered_relative_path)) {
    if (registered_relative_path == requested_relative_path) {
      return std::filesystem::path(package_prefix) / "share" / parts.package / registered_relative_path;
    }
  }

  // Preserve the requested definition path in the eventual filesystem error.
  return std::filesystem::path(package_prefix) / "share" / parts.package / requested_relative_path;
}

std::string loadInterfaceDefinition(const std::string & interface_type)
{
  if (const auto failure = lookupFailureCache().get(interface_type)) {
    std::rethrow_exception(*failure);
  }

  const char * reason = "package_not_found";
  try {
    const TypeParts parts = TypeParts::parse(interface_type);

    const std::filesystem::path path = resolveInterfaceDefinitionPath(parts);
    reason = "definition_file_unavailable";
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open interface definition file: " + path.string());
    }

    std::ostringstream body;
    body << file.rdbuf();
    return body.str();
  } catch (const std::invalid_argument & exc) {
    lookupFailureCache().insertOrAssign(interface_type, std::current_exception());
    LogEvent(kLogger, "interface_definition_lookup_rejected")
      .field("interface_type", interface_type)
      .field("error", exc.what())
      .warn();
    throw;
  } catch (const std::runtime_error & exc) {
    lookupFailureCache().insertOrAssign(interface_type, std::current_exception());
    LogEvent(kLogger, "interface_definition_lookup_failed")
      .field("interface_type", interface_type)
      .field("reason", reason)
      .field("error", exc.what())
      .error();
    throw;
  }
}

bool isPackageLocalMessageType(const std::string & type)
{
  // rosidl_adapter treats only ROS message-shaped unqualified types as package-local messages.
  return !type.empty() && type.front() >= 'A' && type.front() <= 'Z';
}

// Field dependencies are messages; normalize shorthand references to `pkg/msg/Type`.
std::vector<std::string> extractDependencies(const std::string & definition, const std::string & package)
{
  std::vector<std::string> dependencies;
  std::istringstream stream(definition);
  std::string line;

  while (std::getline(stream, line)) {
    line = line.substr(0, line.find('#'));
    const auto first_non_space = line.find_first_not_of(" \t");
    if (
      first_non_space == std::string::npos ||
      line.compare(first_non_space, sizeof(kServiceDefinitionSeparator) - 1U, kServiceDefinitionSeparator) == 0)
    {
      continue;
    }

    std::istringstream fields(line.substr(first_non_space));
    std::string type;
    std::string name;
    fields >> type >> name;
    if (type.empty() || name.empty()) {
      continue;
    }

    // Constants look like `type NAME=...` and do not introduce dependencies.
    if (name.find('=') != std::string::npos) {
      continue;
    }

    const std::string base = type.substr(0, type.find('['));
    std::string dependency;
    const auto first_slash = base.find('/');
    if (first_slash == std::string::npos) {
      if (!isPackageLocalMessageType(base)) {
        continue;
      }
      dependency = package + "/msg/" + base;
    } else if (base.find('/', first_slash + 1) == std::string::npos) {
      dependency = base.substr(0, first_slash) + "/msg/" + base.substr(first_slash + 1);
    } else {
      dependency = base;
    }

    dependencies.push_back(dependency);
  }

  return dependencies;
}

void collectInterfaceDefinitions(
  const std::string & interface_type, std::set<std::string> & visited, std::vector<InterfaceDefinition> & definitions)
{
  if (!visited.insert(interface_type).second) {
    return;
  }

  const std::string body = loadInterfaceDefinition(interface_type);
  definitions.push_back({interface_type, body});

  const TypeParts parts = TypeParts::parse(interface_type);
  for (const auto & dependency : extractDependencies(body, parts.package)) {
    collectInterfaceDefinitions(dependency, visited, definitions);
  }
}

}  // namespace

std::vector<InterfaceDefinition> lookupInterfaceDefinitions(const std::string & interface_type)
{
  std::set<std::string> visited;
  std::vector<InterfaceDefinition> definitions;
  collectInterfaceDefinitions(interface_type, visited, definitions);
  return definitions;
}

}  // namespace livekit_ros2_bridge
