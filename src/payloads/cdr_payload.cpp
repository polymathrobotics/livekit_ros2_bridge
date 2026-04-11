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
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "utils/base64.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kContentTypeField[] = "content_type";
constexpr char kPayloadBase64Field[] = "payload_base64";

std::vector<std::uint8_t> base64Decode(const std::string & value)
{
  const Base64DecodeResult decoded = livekit_ros2_bridge::base64Decode(value);
  switch (decoded.status) {
    case Base64DecodeStatus::kOk:
      return decoded.bytes;
    case Base64DecodeStatus::kMissingPadding:
      throw std::invalid_argument("payload_base64 must be padded standard base64.");
    case Base64DecodeStatus::kInvalidEncoding:
      throw std::invalid_argument("payload_base64 is not valid base64.");
  }

  throw std::invalid_argument("payload_base64 is not valid base64.");
}

const nlohmann::json & requireObjectField(const nlohmann::json & body, const char * field_name)
{
  const auto it = body.find(field_name);
  if (it == body.end() || !it->is_object()) {
    throw std::invalid_argument(std::string(field_name) + " must be an object.");
  }
  return *it;
}

const std::string & requireStringField(const nlohmann::json & body, const char * field_name)
{
  const auto it = body.find(field_name);
  if (it == body.end() || !it->is_string()) {
    throw std::invalid_argument(std::string(field_name) + " must be a string.");
  }
  return it->get_ref<const std::string &>();
}

}  // namespace

std::vector<std::uint8_t> parseCdrPayload(const nlohmann::json & body, const char * field_name)
{
  const auto & field = requireObjectField(body, field_name);
  const std::string & content_type = requireStringField(field, kContentTypeField);
  if (content_type != protocol::kDataContentTypeCdr) {
    throw std::invalid_argument(std::string(field_name) + "." + kContentTypeField + " must be application/x-ros-cdr.");
  }

  const std::string & encoded_payload = requireStringField(field, kPayloadBase64Field);
  return base64Decode(encoded_payload);
}

nlohmann::json serializeCdrPayload(const std::vector<std::uint8_t> & payload)
{
  return {
    {kContentTypeField, protocol::kDataContentTypeCdr},
    {kPayloadBase64Field, base64Encode(payload.data(), payload.size())},
  };
}

}  // namespace livekit_ros2_bridge
