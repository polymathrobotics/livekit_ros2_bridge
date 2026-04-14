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

#include "payloads/cdr_payload.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "payloads/cdr_base64.hpp"
#include "protocol.hpp"

namespace livekit_ros2_bridge::wire::cdr
{

namespace
{

constexpr char kContentTypeField[] = "content_type";
constexpr char kPayloadBase64Field[] = "payload_base64";

const nlohmann::json & requireObject(const nlohmann::json & body, const char * field)
{
  const auto it = body.find(field);
  if (it == body.end() || !it->is_object()) {
    throw std::invalid_argument(std::string(field) + " must be an object.");
  }

  return *it;
}

const std::string & requireString(const nlohmann::json & object, const char * field)
{
  const auto it = object.find(field);
  if (it == object.end() || !it->is_string()) {
    throw std::invalid_argument(std::string(field) + " must be a string.");
  }

  return it->get_ref<const std::string &>();
}

std::vector<std::uint8_t> decodePayload(const std::string & base64)
{
  // The bridge treats padded standard base64 as part of the wire contract so malformed payloads
  // fail here instead of reaching downstream ROS deserialization with ambiguous byte contents.
  auto decoded = decodeBase64(base64);
  if (decoded.status == Base64Status::kOk) {
    return std::move(decoded.bytes);
  }

  if (decoded.status == Base64Status::kMissingPadding) {
    throw std::invalid_argument("payload_base64 must be padded standard base64.");
  }

  throw std::invalid_argument("payload_base64 is not valid base64.");
}

}  // namespace

std::vector<std::uint8_t> parse(const nlohmann::json & body, const char * field)
{
  const auto & envelope = requireObject(body, field);
  if (requireString(envelope, kContentTypeField) != wire::protocol::kDataContentTypeCdr) {
    throw std::invalid_argument(std::string(field) + "." + kContentTypeField + " must be application/x-ros-cdr.");
  }

  return decodePayload(requireString(envelope, kPayloadBase64Field));
}

nlohmann::json serialize(const std::vector<std::uint8_t> & bytes)
{
  return {
    {kContentTypeField, wire::protocol::kDataContentTypeCdr},
    {kPayloadBase64Field, encodeBase64(bytes.data(), bytes.size())},
  };
}

}  // namespace livekit_ros2_bridge::wire::cdr
