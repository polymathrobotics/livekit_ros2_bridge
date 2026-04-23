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
#include <vector>

#include "nlohmann/json.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::wire::detail
{

inline nlohmann::json parseObjectPayload(
  const std::string & payload, const char * invalid_json_message, const char * invalid_object_message)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(payload);
  } catch (const nlohmann::json::exception &) {
    throw std::invalid_argument(invalid_json_message);
  }

  if (!body.is_object()) {
    throw std::invalid_argument(invalid_object_message);
  }

  return body;
}

inline std::optional<std::string> optionalTrimmedString(
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

inline std::string requiredTrimmedString(
  const nlohmann::json & value, const char * invalid_message, const char * empty_message = nullptr)
{
  if (const auto trimmed = optionalTrimmedString(value, invalid_message)) {
    return *trimmed;
  }

  throw std::invalid_argument(empty_message == nullptr ? invalid_message : empty_message);
}

inline std::string requiredTrimmedStringField(
  const nlohmann::json & body, const char * field_name, const char * invalid_message, const char * empty_message)
{
  const auto field = body.find(field_name);
  if (field == body.end()) {
    throw std::invalid_argument(invalid_message);
  }

  return requiredTrimmedString(*field, invalid_message, empty_message);
}

inline std::string requiredTrimmedStringField(
  const nlohmann::json & body, const char * field_name, const char * invalid_message)
{
  return requiredTrimmedStringField(body, field_name, invalid_message, invalid_message);
}

inline std::optional<std::string> optionalTrimmedStringField(
  const nlohmann::json & body, const char * field_name, const char * invalid_message, bool null_is_absent = false)
{
  const auto field = body.find(field_name);
  if (field == body.end()) {
    return std::nullopt;
  }

  return optionalTrimmedString(*field, invalid_message, null_is_absent);
}

// Error messages are synthesized from `field_name` so call sites only need to name the field.
inline std::vector<std::string> requiredTrimmedStringArrayField(const nlohmann::json & body, const char * field_name)
{
  const auto field = body.find(field_name);
  if (field == body.end() || !field->is_array()) {
    throw std::invalid_argument(std::string(field_name) + " must be an array");
  }

  const auto entry_invalid = std::string(field_name) + " entries must be strings";
  const auto entry_empty = std::string(field_name) + " entries must not be empty";

  std::vector<std::string> values;
  values.reserve(field->size());
  for (const auto & entry : *field) {
    values.push_back(requiredTrimmedString(entry, entry_invalid.c_str(), entry_empty.c_str()));
  }

  if (values.empty()) {
    throw std::invalid_argument(std::string(field_name) + " must not be empty");
  }

  return values;
}

}  // namespace livekit_ros2_bridge::wire::detail
