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

#include "wire/detail/base64.hpp"

#include <openssl/evp.h>

#include <stdexcept>
#include <string>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::wire::detail
{

namespace
{

const auto kLogger = rclcpp::get_logger("wire_base64");

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

  std::string text(((size + 2U) / 3U) * 4U, '\0');
  const int encoded = EVP_EncodeBlock(
    reinterpret_cast<unsigned char *>(text.data()),
    reinterpret_cast<const unsigned char *>(bytes),
    static_cast<int>(size));
  if (encoded < 0) {
    LogEvent(kLogger, "base64_encode_failed").field("byte_count", size).error();
    throw std::runtime_error("Failed base64 encoding.");
  }

  text.resize(static_cast<std::size_t>(encoded));
  return text;
}

Base64DecodeResult decodeBase64(std::string_view text)
{
  if (text.empty()) {
    return {};
  }

  const std::size_t size = text.size();
  std::size_t pads = 0U;
  std::size_t pad_index = size;
  // Validate the alphabet and require any '=' padding to form a single suffix before
  // handing the input to OpenSSL so we can distinguish missing padding from other
  // malformed encodings.
  for (std::size_t i = 0; i < size; ++i) {
    const char c = text[i];
    if (c == '=') {
      if (pad_index == size) {
        pad_index = i;
      }
      continue;
    }

    if (pad_index != size || base64Value(c) < 0) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  if (pad_index != size) {
    pads = size - pad_index;
    if (pads > 2U || size == 1U) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  // A structurally valid alphabet/padding sequence that is not quartet-aligned is the
  // specific "missing padding" case callers surface separately at the JSON boundary.
  if ((size % 4U) != 0U) {
    return {{}, Base64Status::kMissingPadding};
  }

  // RFC 4648 requires unused bits in the final sextet(s) to be zero. `EVP_DecodeBlock`
  // decodes successfully either way, so reject non-canonical encodings here.
  if (pads != 0U) {
    const int tail = base64Value(text[size - (pads + 1U)]);
    const int pad_mask = (pads == 2U) ? 0x0F : 0x03;
    if (tail < 0 || (tail & pad_mask) != 0) {
      return {{}, Base64Status::kInvalidEncoding};
    }
  }

  std::vector<std::uint8_t> bytes((size / 4U) * 3U, 0U);
  const int decoded = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(bytes.data()),
    reinterpret_cast<const unsigned char *>(text.data()),
    static_cast<int>(size));
  if (decoded < 0) {
    LogEvent(kLogger, "base64_decode_failed").field("input_chars", size).error();
    return {{}, Base64Status::kInvalidEncoding};
  }

  // `EVP_DecodeBlock` materializes three bytes per quartet; trim the synthetic tail
  // bytes that correspond to validated '=' padding.
  bytes.resize(static_cast<std::size_t>(decoded) - pads);
  return {std::move(bytes), Base64Status::kOk};
}

}  // namespace livekit_ros2_bridge::wire::detail
