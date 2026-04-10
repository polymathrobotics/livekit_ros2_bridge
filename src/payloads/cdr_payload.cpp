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

#include <openssl/evp.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kContentTypeField[] = "content_type";
constexpr char kPayloadBase64Field[] = "payload_base64";

std::string base64Encode(const std::uint8_t * data, std::size_t size)
{
  if (size == 0U) {
    return "";
  }

  const std::size_t encoded_size = ((size + 2U) / 3U) * 4U;
  std::string encoded(encoded_size, '\0');
  const int written = EVP_EncodeBlock(reinterpret_cast<unsigned char *>(encoded.data()), data, static_cast<int>(size));
  if (written < 0) {
    throw std::runtime_error("Failed base64 encoding.");
  }

  encoded.resize(static_cast<std::size_t>(written));
  return encoded;
}

std::vector<std::uint8_t> base64Decode(const std::string & value)
{
  if (value.empty()) {
    return {};
  }

  if ((value.size() % 4U) != 0U) {
    throw std::invalid_argument("payload_base64 must be padded standard base64.");
  }

  const std::size_t max_decoded_size = (value.size() / 4U) * 3U;
  std::vector<std::uint8_t> decoded(max_decoded_size, 0);
  const int written = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(decoded.data()),
    reinterpret_cast<const unsigned char *>(value.data()),
    static_cast<int>(value.size()));
  if (written < 0) {
    throw std::invalid_argument("payload_base64 is not valid base64.");
  }

  std::size_t actual_size = static_cast<std::size_t>(written);
  // EVP_DecodeBlock reports the padded output size, so trim bytes implied only by '=' padding.
  if (!value.empty() && value.back() == '=') {
    --actual_size;
  }
  if (value.size() > 1U && value[value.size() - 2U] == '=') {
    --actual_size;
  }
  decoded.resize(actual_size);
  return decoded;
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
