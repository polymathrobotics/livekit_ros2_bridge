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

#include "payloads/cdr_base64.hpp"

#include <openssl/evp.h>

#include <stdexcept>
#include <string>

namespace livekit_ros2_bridge
{

namespace
{

bool isBase64Char(char ch) noexcept
{
  if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')) {
    return true;
  }
  return ch == '+' || ch == '/';
}

bool hasCanonicalPadding(std::string_view value) noexcept
{
  const auto first_padding = value.find('=');
  if (first_padding == std::string_view::npos) {
    return true;
  }

  for (std::size_t index = first_padding; index < value.size(); ++index) {
    if (value[index] != '=') {
      return false;
    }
  }

  const std::size_t padding_count = value.size() - first_padding;
  return padding_count <= 2U && first_padding >= value.size() - 2U;
}

Base64DecodeResult decodeBase64(std::string_view value)
{
  if (value.empty()) {
    return {};
  }

  if (!hasCanonicalPadding(value)) {
    return {{}, Base64DecodeStatus::kInvalidEncoding};
  }

  for (char ch : value) {
    if (ch != '=' && !isBase64Char(ch)) {
      return {{}, Base64DecodeStatus::kInvalidEncoding};
    }
  }

  if ((value.size() % 4U) != 0U) {
    return {{}, Base64DecodeStatus::kMissingPadding};
  }

  std::vector<std::uint8_t> decoded((value.size() / 4U) * 3U, 0U);
  const int written = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(decoded.data()),
    reinterpret_cast<const unsigned char *>(value.data()),
    static_cast<int>(value.size()));
  if (written < 0) {
    return {{}, Base64DecodeStatus::kInvalidEncoding};
  }

  std::size_t actual_size = static_cast<std::size_t>(written);
  if (!value.empty() && value.back() == '=') {
    --actual_size;
  }
  if (value.size() > 1U && value[value.size() - 2U] == '=') {
    --actual_size;
  }
  decoded.resize(actual_size);
  return {std::move(decoded), Base64DecodeStatus::kOk};
}

std::string encodeBase64(const std::uint8_t * data, std::size_t size)
{
  if (size == 0U) {
    return "";
  }

  std::string encoded(((size + 2U) / 3U) * 4U, '\0');
  const int written = EVP_EncodeBlock(
    reinterpret_cast<unsigned char *>(encoded.data()),
    reinterpret_cast<const unsigned char *>(data),
    static_cast<int>(size));
  if (written < 0) {
    throw std::runtime_error("Failed base64 encoding.");
  }

  encoded.resize(static_cast<std::size_t>(written));
  return encoded;
}

}  // namespace

std::string base64Encode(const std::uint8_t * data, std::size_t size)
{
  return encodeBase64(data, size);
}

Base64DecodeResult base64Decode(std::string_view value)
{
  return decodeBase64(value);
}

}  // namespace livekit_ros2_bridge
