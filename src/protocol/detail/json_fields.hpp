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

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::protocol::detail
{

inline nlohmann::json parseObject(const std::string & payload, const char * invalid_json, const char * invalid_object)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(payload);
  } catch (const nlohmann::json::exception &) {
    throw std::invalid_argument(invalid_json);
  }

  if (!body.is_object()) {
    throw std::invalid_argument(invalid_object);
  }

  return body;
}

inline nlohmann::json parseObject(
  const std::vector<std::uint8_t> & payload, const char * invalid_json, const char * invalid_object)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(payload.begin(), payload.end());
  } catch (const nlohmann::json::exception &) {
    throw std::invalid_argument(invalid_json);
  }

  if (!body.is_object()) {
    throw std::invalid_argument(invalid_object);
  }

  return body;
}

/// Blank strings become absent; null is absent only when `null_absent` is true.
inline std::optional<std::string> optionalTrimmedString(
  const nlohmann::json & value, const char * invalid, bool null_absent = false)
{
  if (null_absent && value.is_null()) {
    return std::nullopt;
  }
  if (!value.is_string()) {
    throw std::invalid_argument(invalid);
  }

  const auto trimmed = trim(value.get_ref<const std::string &>());
  if (trimmed.empty()) {
    return std::nullopt;
  }

  return trimmed;
}

inline std::string requiredTrimmedStringField(
  const nlohmann::json & body, const char * name, const char * invalid, const char * empty = nullptr)
{
  const auto it = body.find(name);
  if (it == body.end()) {
    throw std::invalid_argument(invalid);
  }

  if (const auto trimmed = optionalTrimmedString(*it, invalid)) {
    return *trimmed;
  }

  throw std::invalid_argument(empty == nullptr ? invalid : empty);
}

inline std::optional<std::string> optionalTrimmedStringField(
  const nlohmann::json & body, const char * name, const char * invalid, bool null_absent = false)
{
  const auto it = body.find(name);
  if (it == body.end()) {
    return std::nullopt;
  }

  return optionalTrimmedString(*it, invalid, null_absent);
}

}  // namespace livekit_ros2_bridge::protocol::detail
