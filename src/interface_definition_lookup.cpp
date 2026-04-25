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
#include "utils/lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kSectionSeparator[] = "---";
constexpr char kAmentResourceType[] = "rosidl_interfaces";
constexpr std::size_t kFailureCacheCapacity = 256U;

[[noreturn]] void throwInvalidType(const std::string & type, const char * reason)
{
  throw std::invalid_argument("Invalid ROS interface type '" + type + "': " + reason);
}

struct TypeParts
{
  std::string package;
  std::string kind;
  std::string name;

  static TypeParts parse(const std::string & type)
  {
    const auto first_slash = type.find('/');
    if (first_slash == std::string::npos) {
      throwInvalidType(type, "expected package/kind/Name");
    }

    const auto second_slash = type.find('/', first_slash + 1);
    if (second_slash == std::string::npos) {
      throwInvalidType(type, "expected package/kind/Name");
    }

    const std::string package = type.substr(0, first_slash);
    const std::string kind = type.substr(first_slash + 1, second_slash - first_slash - 1);
    const std::string name = type.substr(second_slash + 1);
    if (package.empty() || kind.empty() || name.empty()) {
      throwInvalidType(type, "empty component");
    }
    if (name.find('/') != std::string::npos) {
      throwInvalidType(type, "expected package/kind/Name");
    }
    if (kind != "msg" && kind != "srv" && kind != "action") {
      throwInvalidType(type, "kind must be msg, srv, or action");
    }

    return {package, kind, name};
  }
};

std::filesystem::path resolvePath(const TypeParts & parts)
{
  std::string index;
  std::string prefix;
  if (!ament_index_cpp::get_resource(kAmentResourceType, parts.package, index, &prefix)) {
    throw std::runtime_error("Package '" + parts.package + "' not found in ament index");
  }

  const std::string requested_path = parts.kind + "/" + parts.name + "." + parts.kind;
  std::istringstream lines(index);
  std::string registered_path;
  while (std::getline(lines, registered_path)) {
    if (registered_path == requested_path) {
      return std::filesystem::path(prefix) / "share" / parts.package / registered_path;
    }
  }

  // Preserve the requested definition path in the eventual filesystem error.
  return std::filesystem::path(prefix) / "share" / parts.package / requested_path;
}

std::string loadDefinition(const std::string & type)
{
  static LruCache<std::string, std::exception_ptr> failures(kFailureCacheCapacity);

  if (const auto failure = failures.get(type)) {
    std::rethrow_exception(*failure);
  }

  try {
    const TypeParts parts = TypeParts::parse(type);

    const std::filesystem::path path = resolvePath(parts);
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open interface definition file: " + path.string());
    }

    std::ostringstream body;
    body << file.rdbuf();
    return body.str();
  } catch (const std::invalid_argument &) {
    failures.set(type, std::current_exception());
    throw;
  } catch (const std::runtime_error &) {
    failures.set(type, std::current_exception());
    throw;
  }
}

// Field dependencies are messages; normalize shorthand references to `pkg/msg/Type`.
std::vector<std::string> parseDependencies(const std::string & definition, const std::string & package)
{
  std::vector<std::string> dependencies;
  std::istringstream stream(definition);
  std::string line;

  while (std::getline(stream, line)) {
    line = line.substr(0, line.find('#'));
    const auto first_non_space = line.find_first_not_of(" \t");
    if (
      first_non_space == std::string::npos ||
      line.compare(first_non_space, sizeof(kSectionSeparator) - 1U, kSectionSeparator) == 0)
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
      // rosidl_adapter treats only ROS message-shaped unqualified types as package-local messages.
      if (base.empty() || base.front() < 'A' || base.front() > 'Z') {
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

void collectDefinitions(
  const std::string & type, std::set<std::string> & seen, std::vector<InterfaceDefinition> & definitions)
{
  if (!seen.insert(type).second) {
    return;
  }

  const std::string body = loadDefinition(type);
  definitions.push_back({type, body});

  const TypeParts parts = TypeParts::parse(type);
  for (const auto & dependency : parseDependencies(body, parts.package)) {
    collectDefinitions(dependency, seen, definitions);
  }
}

}  // namespace

std::vector<InterfaceDefinition> lookupDefinitions(const std::string & type)
{
  std::set<std::string> seen;
  std::vector<InterfaceDefinition> definitions;
  collectDefinitions(type, seen, definitions);
  return definitions;
}

}  // namespace livekit_ros2_bridge
