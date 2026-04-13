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

#pragma once

#include <functional>
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

/// One ROS interface definition entry returned to a remote caller.
/// Wire-format note: interface_payloads::serialize() maps these members directly to the
/// `ros.interfaces.get` JSON response keys. Treat those keys and their meanings as public
/// client-facing contract; do not rename or repurpose them without an explicit compatibility plan.
struct InterfaceDefinition
{
  /// Fully-qualified ROS interface type such as `sensor_msgs/msg/BatteryState`.
  std::string interface_type;
  /// Stable format label for `definition`.
  std::string format;
  /// Raw `.msg`, `.srv`, or `.action` file contents as read from the ROS package share directory.
  std::string definition;
};

/// Look up a fully-qualified ROS interface type and read its `.msg`, `.srv`, or `.action`
/// definition plus any transitive message dependencies. The requested definition is always
/// returned first, followed by unique dependencies in first-discovery order during recursive
/// traversal.
/// Throws `std::invalid_argument` for malformed `package/kind/Name` identifiers and
/// `std::runtime_error` when the package or definition file cannot be found. Failures are
/// memoized process-locally by interface type so repeated bad requests fail fast.
std::vector<InterfaceDefinition> lookupInterfaceDefinitions(const std::string & interface_type);

// Test-only helpers for validating negative-cache behavior. They mutate process-global lookup
// state, so use them only from tests that own `lookupInterfaceDefinitions()` for their duration.
// The hook fires only on uncached lookup attempts, and
// `resetInterfaceLookupForTest()` clears both the failure cache and any installed hook.
void setInterfaceLookupAttemptHookForTest(std::function<void(const std::string &)> hook);
void resetInterfaceLookupForTest();

}  // namespace livekit_ros2_bridge
