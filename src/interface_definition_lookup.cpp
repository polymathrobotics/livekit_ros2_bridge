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
#include <functional>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/version.h"
#include "utils/log_event.hpp"
#include "utils/lru_cache.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kDefinitionFormatRos2Msg[] = "ros2msg";
constexpr char kServiceDefinitionSeparator[] = "---";
constexpr std::size_t kInvalidTypeCacheCapacity = 256U;
const auto kLogger = rclcpp::get_logger("interface_definition_lookup");

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

using LookupFailureCache = LruCache<std::string, std::exception_ptr>;

LookupFailureCache & lookupFailureCache()
{
  // Preserve the original exception type/message for repeated bad interface requests without
  // re-hitting the ament index or filesystem each time.
  static LookupFailureCache cache(kInvalidTypeCacheCapacity);
  return cache;
}

// Test-only observability for uncached lookups. This is process-global like the negative cache, so
// tests install/clear it around isolated lookup sequences rather than treating it as per-call
// state.
std::mutex attempt_hook_mutex;
std::function<void(const std::string &)> attempt_hook;

[[noreturn]] void throwInvalidInterfaceType(const std::string & interface_type, const char * reason)
{
  throw std::invalid_argument("Invalid ROS interface type '" + interface_type + "': " + reason);
}

// Parse once so validation, path lookup, and dependency resolution share the same identifier parts.
struct InterfaceTypeParts
{
  std::string package;
  std::string kind;
  std::string name;

  static InterfaceTypeParts parse(const std::string & interface_type)
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

// Minimal lookup result carried forward into dependency traversal.
struct ResolvedInterfaceDefinition
{
  std::string package;
  std::string definition;
};

std::string getPackageShareDirCompat(const std::string & package)
{
  // ament_index_cpp 1.11+ adds the filesystem-path overload used by Kilted/Rolling, while
  // Humble and Jazzy still only expose the legacy string-returning API. Keep one compatibility
  // wrapper here instead of scattering distro/version checks across interface definition lookup
  // code.
#if AMENT_INDEX_CPP_VERSION_GTE(1, 11, 0)
  std::filesystem::path share_dir;
  ament_index_cpp::get_package_share_directory(package, share_dir);
  return share_dir.string();
#else
  return ament_index_cpp::get_package_share_directory(package);
#endif
}

void notifyLookupAttempt(const std::string & interface_type)
{
  std::function<void(const std::string &)> hook;
  {
    // Copy the hook while holding the mutex, then invoke it after releasing the lock so tests can
    // replace or clear the hook from inside callbacks without self-deadlocking.
    std::lock_guard<std::mutex> lock(attempt_hook_mutex);
    hook = attempt_hook;
  }
  if (hook) {
    hook(interface_type);
  }
}

ResolvedInterfaceDefinition loadInterfaceDefinition(const std::string & interface_type)
{
  // Keep the uncached lookup path together so traversal only has to reason about ordering and
  // de-duplication.
  if (const auto failure = lookupFailureCache().get(interface_type); failure.has_value()) {
    std::rethrow_exception(*failure);
  }

  notifyLookupAttempt(interface_type);

  const char * failure_reason = "lookup_runtime_error";
  try {
    const InterfaceTypeParts parts = InterfaceTypeParts::parse(interface_type);

    std::string share_dir;
    try {
      share_dir = getPackageShareDirCompat(parts.package);
    } catch (const std::exception &) {
      failure_reason = "package_not_found";
      throw std::runtime_error("Package '" + parts.package + "' not found in ament index");
    }

    const std::filesystem::path definition_path =
      std::filesystem::path(share_dir) / parts.kind / (parts.name + "." + parts.kind);
    std::ifstream definition_file(definition_path);
    if (!definition_file.is_open()) {
      failure_reason = "definition_file_unavailable";
      throw std::runtime_error("Cannot open interface definition file: " + definition_path.string());
    }

    std::ostringstream contents;
    contents << definition_file.rdbuf();
    return {parts.package, contents.str()};
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
      .field("reason", failure_reason)
      .field("error", exc.what())
      .error();
    throw;
  }
}

/// Scan a .msg, .srv, or .action file for complex type references and return their fully-qualified
/// names.
/// Field types are always messages, so short-form references like "std_msgs/Header" are
/// qualified as "std_msgs/msg/Header" regardless of whether the containing file is msg, srv, or
/// action.
std::vector<std::string> extractDependencies(const std::string & definition, const std::string & package)
{
  std::vector<std::string> dependencies;
  std::istringstream stream(definition);
  std::string line;

  while (std::getline(stream, line)) {
    line = line.substr(0, line.find('#'));
    const auto first_non_space = line.find_first_not_of(" \t");
    if (first_non_space == std::string::npos) {
      continue;
    }

    if (line.compare(first_non_space, sizeof(kServiceDefinitionSeparator) - 1U, kServiceDefinitionSeparator) == 0) {
      continue;
    }

    std::istringstream line_stream(line.substr(first_non_space));
    std::string type_token;
    std::string name_token;
    line_stream >> type_token >> name_token;
    if (type_token.empty() || name_token.empty()) {
      continue;
    }

    // Constant declarations look like `type NAME=...`; they do not introduce dependent types.
    if (name_token.find('=') != std::string::npos) {
      continue;
    }

    const std::string base_type = type_token.substr(0, type_token.find('['));
    if (kPrimitiveTypes.count(base_type) > 0) {
      continue;
    }

    const auto first_slash = base_type.find('/');
    if (first_slash == std::string::npos) {
      dependencies.push_back(package + "/msg/" + base_type);
      continue;
    }

    const auto second_slash = base_type.find('/', first_slash + 1);
    if (second_slash != std::string::npos) {
      dependencies.push_back(base_type);
      continue;
    }

    dependencies.push_back(base_type.substr(0, first_slash) + "/msg/" + base_type.substr(first_slash + 1));
  }

  return dependencies;
}

// Depth-first walk that preserves response order by appending each interface at first discovery
// before recursing into referenced message types. `visited` de-duplicates shared dependencies
// across nested messages, services, and actions without disturbing that first-discovery order.
void collectInterfaceDefinitions(
  const std::string & interface_type, std::set<std::string> & visited, std::vector<InterfaceDefinition> & definitions)
{
  if (visited.count(interface_type) > 0) {
    return;
  }
  visited.insert(interface_type);

  const ResolvedInterfaceDefinition resolved = loadInterfaceDefinition(interface_type);
  definitions.push_back({interface_type, kDefinitionFormatRos2Msg, resolved.definition});

  for (const auto & dependency : extractDependencies(resolved.definition, resolved.package)) {
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

void setInterfaceLookupAttemptHookForTest(std::function<void(const std::string &)> hook)
{
  std::lock_guard<std::mutex> lock(attempt_hook_mutex);
  attempt_hook = std::move(hook);
}

void resetInterfaceLookupForTest()
{
  lookupFailureCache().clear();

  std::lock_guard<std::mutex> lock(attempt_hook_mutex);
  attempt_hook = nullptr;
}

}  // namespace livekit_ros2_bridge
