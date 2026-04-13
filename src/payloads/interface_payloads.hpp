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

#include <string>
#include <vector>

#include "interface_definition_lookup.hpp"

namespace livekit_ros2_bridge
{

namespace interface_payloads
{

/// Parse an interfaces-get request body of the form
/// `{ "interface_types": ["pkg/msg/Type", ...] }`.
/// Entries are trimmed, must stay non-empty, and are returned in request order.
/// Duplicate entries are preserved; de-duplication, if desired, happens at the caller boundary.
/// Unrelated top-level fields are ignored so the request envelope can grow without changing this parser.
/// Throws `std::invalid_argument` with caller-fixable validation text that is surfaced through
/// the RPC invalid-request path.
std::vector<std::string> parse(const std::string & payload);

/// Serialize interface definitions as
/// `{ "interfaces": [{ "interface_type", "format", "definition" }, ...] }`
/// while preserving the input order used by the caller to express dependency/result ordering.
std::string serialize(const std::vector<InterfaceDefinition> & definitions);

// TODO: Remove these compatibility wrappers once remaining call sites migrate to
// `interface_payloads::parse` / `interface_payloads::serialize`.
inline std::vector<std::string> parseRequest(const std::string & payload)
{
  return parse(payload);
}

inline std::string serializeResponse(const std::vector<InterfaceDefinition> & definitions)
{
  return serialize(definitions);
}

}  // namespace interface_payloads

// TODO: Remove these compatibility wrappers once remaining external call sites migrate to
// `interface_payloads::parse` / `interface_payloads::serialize`.
inline std::vector<std::string> parseInterfaceTypes(const std::string & payload)
{
  return interface_payloads::parse(payload);
}

inline std::string serializeInterfaces(const std::vector<InterfaceDefinition> & definitions)
{
  return interface_payloads::serialize(definitions);
}

}  // namespace livekit_ros2_bridge
