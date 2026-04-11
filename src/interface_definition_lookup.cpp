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

#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/version.h"
#include "utils/bounded_lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kSchemaEncodingRos2Msg[] = "ros2msg";
constexpr char kServiceDefinitionSeparator[] = "---";
constexpr std::size_t kInvalidInterfaceTypeCacheCapacity = 256U;

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

using FailureCache = BoundedLruCache<std::string, std::exception_ptr>;

FailureCache & invalidInterfaceTypeCache()
{
  static FailureCache cache(kInvalidInterfaceTypeCacheCapacity);
  return cache;
}

std::mutex & interfaceDefinitionLookupAttemptHookMutex()
{
  static std::mutex mutex;
  return mutex;
}

std::function<void(const std::string &)> & interfaceDefinitionLookupAttemptHook()
{
  static std::function<void(const std::string &)> hook;
  return hook;
}

void noteInterfaceDefinitionLookupAttempt(const std::string & interface_type)
{
  std::function<void(const std::string &)> hook;
  {
    std::lock_guard<std::mutex> lock(interfaceDefinitionLookupAttemptHookMutex());
    hook = interfaceDefinitionLookupAttemptHook();
  }
  if (hook) {
    hook(interface_type);
  }
}

[[noreturn]] void throwInvalidInterfaceType(const std::string & interface_type, const char * reason)
{
  throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': " + reason);
}

std::string getPackageShareDirectoryCompat(const std::string & package)
{
  // ament_index_cpp 1.11+ adds the filesystem-path overload used by Kilted/Rolling, while
  // Humble and Jazzy still only expose the legacy string-returning API. Keep one compatibility
  // wrapper here instead of scattering distro/version checks across schema lookup code.
#if AMENT_INDEX_CPP_VERSION_GTE(1, 11, 0)
  std::filesystem::path share_dir;
  ament_index_cpp::get_package_share_directory(package, share_dir);
  return share_dir.string();
#else
  return ament_index_cpp::get_package_share_directory(package);
#endif
}

std::string resolveInterfaceDefinitionPath(const std::string & interface_type)
{
  // Expected: "package/kind/Name" where kind is msg, srv, or action
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

  std::string share_dir;
  try {
    share_dir = getPackageShareDirectoryCompat(package);
  } catch (const std::exception &) {
    throw std::runtime_error("Package '" + package + "' not found in ament index");
  }
  const std::string relative_schema_path = kind + "/" + name + "." + kind;
  return share_dir + "/" + relative_schema_path;
}

std::string loadInterfaceDefinition(const std::string & interface_type)
{
  if (const auto failure = invalidInterfaceTypeCache().get(interface_type); failure.has_value()) {
    std::rethrow_exception(*failure);
  }
  noteInterfaceDefinitionLookupAttempt(interface_type);

  try {
    const std::string path = resolveInterfaceDefinitionPath(interface_type);
    return readInterfaceDefinitionFile(path);
  } catch (const std::invalid_argument &) {
    invalidInterfaceTypeCache().insertOrAssign(interface_type, std::current_exception());
    throw;
  } catch (const std::runtime_error &) {
    invalidInterfaceTypeCache().insertOrAssign(interface_type, std::current_exception());
    throw;
  }
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
    if (line.compare(first_non_space, sizeof(kServiceDefinitionSeparator) - 1U, kServiceDefinitionSeparator) == 0) {
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

  const std::string definition = loadInterfaceDefinition(interface_type);

  // Record each schema before recursing so callers can keep the requested type first and the
  // remaining entries in the same first-discovery order used for dependency traversal.
  dependencies.push_back({interface_type, kSchemaEncodingRos2Msg, definition});

  for (const auto & ref : extractTypeReferences(definition)) {
    collectDependencies(ref, visited, dependencies);
  }
}

}  // namespace

std::vector<InterfaceDefinition> lookupInterfaceDefinitions(const std::string & interface_type)
{
  std::set<std::string> visited;
  std::vector<InterfaceDefinition> entries;
  collectDependencies(interface_type, visited, entries);
  return entries;
}

void setInterfaceDefinitionLookupAttemptHookForTest(std::function<void(const std::string &)> hook)
{
  std::lock_guard<std::mutex> lock(interfaceDefinitionLookupAttemptHookMutex());
  interfaceDefinitionLookupAttemptHook() = std::move(hook);
}

void resetInterfaceDefinitionLookupStateForTest()
{
  invalidInterfaceTypeCache().clear();

  std::lock_guard<std::mutex> lock(interfaceDefinitionLookupAttemptHookMutex());
  interfaceDefinitionLookupAttemptHook() = nullptr;
}

}  // namespace livekit_ros2_bridge
