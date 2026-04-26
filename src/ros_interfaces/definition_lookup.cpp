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

#include "ros_interfaces/definition_lookup.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_resource.hpp"
#include "ros_interfaces/failure_cache.hpp"

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

constexpr char kSectionSeparator[] = "---";
constexpr char kAmentResourceType[] = "rosidl_interfaces";
constexpr std::size_t kFailureCapacity = 256U;

[[noreturn]] void throwInvalidType(const std::string & type, const char * reason)
{
  throw std::invalid_argument("Invalid ROS interface type '" + type + "': " + reason);
}

struct ParsedType
{
  std::string package;
  std::string kind;
  std::string name;

  static ParsedType parse(const std::string & type)
  {
    const auto package_end = type.find('/');
    if (package_end == std::string::npos) {
      throwInvalidType(type, "expected package/kind/Name");
    }

    const auto kind_end = type.find('/', package_end + 1);
    if (kind_end == std::string::npos) {
      throwInvalidType(type, "expected package/kind/Name");
    }

    const std::string package = type.substr(0, package_end);
    const std::string kind = type.substr(package_end + 1, kind_end - package_end - 1);
    const std::string name = type.substr(kind_end + 1);
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

std::filesystem::path resolveDefinitionPath(const ParsedType & parsed)
{
  std::string index;
  std::string prefix;
  if (!ament_index_cpp::get_resource(kAmentResourceType, parsed.package, index, &prefix)) {
    throw std::runtime_error("Package '" + parsed.package + "' not found in ament index");
  }

  const std::string requested_path = parsed.kind + "/" + parsed.name + "." + parsed.kind;
  std::istringstream lines(index);
  std::string registered_path;
  while (std::getline(lines, registered_path)) {
    if (registered_path == requested_path) {
      return std::filesystem::path(prefix) / "share" / parsed.package / registered_path;
    }
  }

  // Preserve the requested definition path in the eventual filesystem error.
  return std::filesystem::path(prefix) / "share" / parsed.package / requested_path;
}

std::string load(const std::string & type)
{
  static FailureCache failures(kFailureCapacity);

  if (const auto failure = failures.find(type)) {
    std::rethrow_exception(*failure);
  }

  try {
    const ParsedType parsed = ParsedType::parse(type);

    const std::filesystem::path path = resolveDefinitionPath(parsed);
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open interface definition file: " + path.string());
    }

    std::ostringstream body;
    body << file.rdbuf();
    return body.str();
  } catch (const std::invalid_argument &) {
    failures.remember(type, std::current_exception());
    throw;
  } catch (const std::runtime_error &) {
    failures.remember(type, std::current_exception());
    throw;
  }
}

// Field dependencies are messages; normalize shorthand references to `pkg/msg/Type`.
std::vector<std::string> parseDependencies(const std::string & body, const std::string & package)
{
  std::vector<std::string> dependencies;
  std::istringstream stream(body);
  std::string line;

  while (std::getline(stream, line)) {
    line = line.substr(0, line.find('#'));
    const auto content = line.find_first_not_of(" \t");
    if (content == std::string::npos || line.compare(content, sizeof(kSectionSeparator) - 1U, kSectionSeparator) == 0) {
      continue;
    }

    std::istringstream tokens(line.substr(content));
    std::string type;
    std::string name;
    tokens >> type >> name;
    if (type.empty() || name.empty()) {
      continue;
    }

    // Constants look like `type NAME=...` and do not introduce dependencies.
    if (name.find('=') != std::string::npos) {
      continue;
    }

    const std::string base = type.substr(0, type.find('['));
    std::string dependency;
    const auto package_end = base.find('/');
    if (package_end == std::string::npos) {
      // rosidl_adapter treats only ROS message-shaped unqualified types as package-local messages.
      if (base.empty() || base.front() < 'A' || base.front() > 'Z') {
        continue;
      }
      dependency = package + "/msg/" + base;
    } else if (base.find('/', package_end + 1) == std::string::npos) {
      dependency = base.substr(0, package_end) + "/msg/" + base.substr(package_end + 1);
    } else {
      dependency = base;
    }

    dependencies.push_back(dependency);
  }

  return dependencies;
}

void collect(const std::string & type, std::set<std::string> & seen, std::vector<InterfaceDefinition> & definitions)
{
  if (!seen.insert(type).second) {
    return;
  }

  const std::string body = load(type);
  definitions.push_back({type, body});

  const ParsedType parsed = ParsedType::parse(type);
  for (const auto & dependency : parseDependencies(body, parsed.package)) {
    collect(dependency, seen, definitions);
  }
}

}  // namespace

std::vector<InterfaceDefinition> lookupDefinitions(const std::string & type)
{
  std::set<std::string> seen;
  std::vector<InterfaceDefinition> definitions;
  collect(type, seen, definitions);
  return definitions;
}

}  // namespace livekit_ros2_bridge::ros_interfaces
