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

// `EVP_DecodeBlock` accepts some non-canonical inputs, so we keep a local alphabet map
// to validate pad placement and trailing pad bits before handing data to OpenSSL.
int base64Value(char c) noexcept
{
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  }
  if (c >= 'a' && c <= 'z') {
    return 26 + (c - 'a');
  }
  if (c >= '0' && c <= '9') {
    return 52 + (c - '0');
  }
  if (c == '+') {
    return 62;
  }
  if (c == '/') {
    return 63;
  }
  return -1;
}

}  // namespace

std::string encodeBase64(const std::uint8_t * bytes, std::size_t size)
{
  if (size == 0U) {
    return "";
  }

  std::string base64(((size + 2U) / 3U) * 4U, '\0');
  const int encoded_size = EVP_EncodeBlock(
    reinterpret_cast<unsigned char *>(base64.data()),
    reinterpret_cast<const unsigned char *>(bytes),
    static_cast<int>(size));
  if (encoded_size < 0) {
    throw std::runtime_error("Failed base64 encoding.");
  }

  base64.resize(static_cast<std::size_t>(encoded_size));
  return base64;
}

Base64DecodeResult decodeBase64(std::string_view base64)
{
  if (base64.empty()) {
    return {};
  }

  std::size_t pad_count = 0U;
  std::size_t pad_start = base64.size();
  // Validate the alphabet and require any '=' padding to form a single suffix before
  // handing the input to OpenSSL so we can distinguish missing padding from other
  // malformed encodings.
  for (std::size_t index = 0; index < base64.size(); ++index) {
    const char c = base64[index];
    if (c == '=') {
      if (pad_start == base64.size()) {
        pad_start = index;
      }
      continue;
    }

    if (pad_start != base64.size() || base64Value(c) < 0) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  if (pad_start != base64.size()) {
    pad_count = base64.size() - pad_start;
    if (pad_count > 2U || base64.size() == 1U) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  // A structurally valid alphabet/padding sequence that is not quartet-aligned is the
  // specific "missing padding" case callers surface separately at the JSON boundary.
  if ((base64.size() % 4U) != 0U) {
    return {{}, Base64Status::kMissingPadding};
  }

  // RFC 4648 requires unused bits in the final sextet(s) to be zero. `EVP_DecodeBlock`
  // decodes successfully either way, so reject non-canonical encodings here.
  if (pad_count == 2U) {
    const int second_sextet = base64Value(base64[base64.size() - 3U]);
    if (second_sextet < 0 || (second_sextet & 0x0F) != 0) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  } else if (pad_count == 1U) {
    const int third_sextet = base64Value(base64[base64.size() - 2U]);
    if (third_sextet < 0 || (third_sextet & 0x03) != 0) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  std::vector<std::uint8_t> bytes((base64.size() / 4U) * 3U, 0U);
  const int decoded_size = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(bytes.data()),
    reinterpret_cast<const unsigned char *>(base64.data()),
    static_cast<int>(base64.size()));
  if (decoded_size < 0) {
    return {{}, Base64Status::kInvalidEncoding};
  }

  // `EVP_DecodeBlock` materializes three bytes per quartet; trim the synthetic tail
  // bytes that correspond to validated '=' padding.
  bytes.resize(static_cast<std::size_t>(decoded_size) - pad_count);
  return {std::move(bytes), Base64Status::kOk};
}

}  // namespace livekit_ros2_bridge
