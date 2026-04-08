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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol.hpp"

namespace livekit_ros2_bridge
{

namespace
{

std::string base64Encode(const std::uint8_t * data, std::size_t size)
{
  if (size == 0U) {
    return "";
  }

  std::string encoded(((size + 2U) / 3U) * 4U, '\0');
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

  std::vector<std::uint8_t> decoded((value.size() / 4U) * 3U, 0);
  const int written = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(decoded.data()),
    reinterpret_cast<const unsigned char *>(value.data()),
    static_cast<int>(value.size()));
  if (written < 0) {
    throw std::invalid_argument("payload_base64 is not valid base64.");
  }

  std::size_t actual_size = static_cast<std::size_t>(written);
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
  const std::string & content_type = requireStringField(field, "content_type");
  if (content_type != protocol::kDataContentTypeCdr) {
    throw std::invalid_argument(std::string(field_name) + ".content_type must be application/x-ros-cdr.");
  }

  return base64Decode(requireStringField(field, "payload_base64"));
}

nlohmann::json serializeCdrPayload(const std::vector<std::uint8_t> & payload)
{
  return {
    {"content_type", protocol::kDataContentTypeCdr},
    {"payload_base64", base64Encode(payload.data(), payload.size())},
  };
}

nlohmann::json serializeCdrPayload(const rclcpp::SerializedMessage & payload)
{
  return serializeCdrPayload(serializedMessageBytes(payload));
}

rclcpp::SerializedMessage toSerializedMessage(const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  if (!payload.empty()) {
    std::memcpy(rcl_msg.buffer, payload.data(), payload.size());
  }
  rcl_msg.buffer_length = payload.size();
  return serialized;
}

std::vector<std::uint8_t> serializedMessageBytes(const rclcpp::SerializedMessage & payload)
{
  const auto & rcl_msg = payload.get_rcl_serialized_message();
  if (rcl_msg.buffer == nullptr || rcl_msg.buffer_length == 0U) {
    return {};
  }
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

}  // namespace livekit_ros2_bridge
