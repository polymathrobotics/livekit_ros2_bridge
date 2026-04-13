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

namespace livekit_ros2_bridge
{

namespace cdr_payload
{

namespace
{

constexpr char kContentTypeField[] = "content_type";
constexpr char kPayloadBase64Field[] = "payload_base64";

const nlohmann::json & requireObjectField(const nlohmann::json & body, const char * field)
{
  const auto field_it = body.find(field);
  if (field_it == body.end() || !field_it->is_object()) {
    throw std::invalid_argument(std::string(field) + " must be an object.");
  }

  return *field_it;
}

const std::string & requireStringField(const nlohmann::json & object, const char * field)
{
  const auto field_it = object.find(field);
  if (field_it == object.end() || !field_it->is_string()) {
    throw std::invalid_argument(std::string(field) + " must be a string.");
  }

  return field_it->get_ref<const std::string &>();
}

std::vector<std::uint8_t> decodePayloadBase64(const std::string & payload_base64)
{
  // The bridge treats padded standard base64 as part of the wire contract so malformed payloads
  // fail here instead of reaching downstream ROS deserialization with ambiguous byte contents.
  auto decoded = decodeBase64(payload_base64);
  switch (decoded.status) {
    case Base64Status::kOk:
      return std::move(decoded.bytes);
    case Base64Status::kMissingPadding:
      throw std::invalid_argument("payload_base64 must be padded standard base64.");
    case Base64Status::kInvalidEncoding:
      throw std::invalid_argument("payload_base64 is not valid base64.");
  }

  throw std::invalid_argument("payload_base64 is not valid base64.");
}

}  // namespace

std::vector<std::uint8_t> parse(const nlohmann::json & body, const char * field)
{
  const auto & envelope = requireObjectField(body, field);
  const auto & type = requireStringField(envelope, kContentTypeField);
  if (type != protocol::kDataContentTypeCdr) {
    throw std::invalid_argument(std::string(field) + "." + kContentTypeField + " must be application/x-ros-cdr.");
  }

  return decodePayloadBase64(requireStringField(envelope, kPayloadBase64Field));
}

nlohmann::json serialize(const std::vector<std::uint8_t> & bytes)
{
  return {
    {kContentTypeField, protocol::kDataContentTypeCdr},
    {kPayloadBase64Field, encodeBase64(bytes.data(), bytes.size())},
  };
}

}  // namespace cdr_payload

}  // namespace livekit_ros2_bridge
