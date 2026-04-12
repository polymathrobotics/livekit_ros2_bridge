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

#include <optional>
#include <stdexcept>
#include <string>

#include "nlohmann/json.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge
{

/// Parse a JSON string value, trim surrounding whitespace, and return `std::nullopt` when the
/// trimmed result is empty. When `null_is_absent` is true, JSON null is also treated as absent.
inline std::optional<std::string> parseOptionalNonEmptyTrimmedString(
  const nlohmann::json & value, const char * invalid_message, bool null_is_absent = false)
{
  if (null_is_absent && value.is_null()) {
    return std::nullopt;
  }
  if (!value.is_string()) {
    throw std::invalid_argument(invalid_message);
  }

  const auto trimmed = trim(value.get_ref<const std::string &>());
  if (trimmed.empty()) {
    return std::nullopt;
  }

  return trimmed;
}

/// Parse a JSON string value, trim surrounding whitespace, and require the trimmed result to stay
/// non-empty.
inline std::string parseRequiredNonEmptyTrimmedString(
  const nlohmann::json & value, const char * invalid_message, const char * empty_message = nullptr)
{
  if (const auto trimmed = parseOptionalNonEmptyTrimmedString(value, invalid_message)) {
    return *trimmed;
  }

  throw std::invalid_argument(empty_message == nullptr ? invalid_message : empty_message);
}

/// Parse `payload` as JSON and require the top-level value to be an object.
/// The supplied error strings are forwarded unchanged when parsing fails or the root is not an object.
inline nlohmann::json parseJsonObject(
  const std::string & payload, const char * invalid_json_message, const char * invalid_object_message)
{
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(payload);
  } catch (const nlohmann::json::exception &) {
    throw std::invalid_argument(invalid_json_message);
  }

  if (!json.is_object()) {
    throw std::invalid_argument(invalid_object_message);
  }

  return json;
}

/// Read a required string field, trim surrounding whitespace, and require the trimmed result
/// to stay non-empty.
inline std::string parseRequiredNonEmptyTrimmedStringField(
  const nlohmann::json & json, const char * field_name, const char * required_message)
{
  const auto field_it = json.find(field_name);
  if (field_it == json.end()) {
    throw std::invalid_argument(required_message);
  }

  return parseRequiredNonEmptyTrimmedString(*field_it, required_message);
}

/// Read an optional string field and return the trimmed value when non-empty.
/// Missing fields always return `std::nullopt`; blank strings also return `std::nullopt`.
/// When `null_is_absent` is true, a JSON null is treated the same as a missing field.
inline std::optional<std::string> parseOptionalNonEmptyTrimmedStringField(
  const nlohmann::json & json, const char * field_name, const char * invalid_message, bool null_is_absent = false)
{
  const auto field_it = json.find(field_name);
  if (field_it == json.end()) {
    return std::nullopt;
  }

  return parseOptionalNonEmptyTrimmedString(*field_it, invalid_message, null_is_absent);
}

}  // namespace livekit_ros2_bridge
