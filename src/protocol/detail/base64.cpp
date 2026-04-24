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

#include "protocol/detail/base64.hpp"

#include <openssl/evp.h>

#include <stdexcept>
#include <string>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::protocol::detail::base64
{

namespace
{

const auto kLogger = rclcpp::get_logger("protocol_base64");

// `EVP_DecodeBlock` accepts some non-canonical inputs, so we keep a local alphabet map
// to validate pad placement and trailing pad bits before handing data to OpenSSL.
int sextetValue(char c) noexcept
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

std::string encode(const std::uint8_t * bytes, std::size_t size)
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

Result decode(std::string_view text)
{
  if (text.empty()) {
    return {};
  }

  const std::size_t size = text.size();
  std::size_t padding = 0U;
  // Validate the alphabet and count any '=' suffix before handing the input to OpenSSL
  // so we can distinguish missing padding from other malformed encodings.
  for (const char c : text) {
    if (c == '=') {
      ++padding;
      continue;
    }

    if (padding != 0U || sextetValue(c) < 0) {
      return {{}, Status::InvalidEncoding};
    }
  }

  if (padding > 2U || (padding != 0U && size == 1U)) {
    return {{}, Status::InvalidEncoding};
  }

  // A structurally valid alphabet/padding sequence that is not quartet-aligned is the
  // specific "missing padding" case callers surface separately at the JSON boundary.
  if ((size % 4U) != 0U) {
    return {{}, Status::MissingPadding};
  }

  // RFC 4648 requires unused bits in the final sextet(s) to be zero. `EVP_DecodeBlock`
  // decodes successfully either way, so reject non-canonical encodings here.
  if (padding != 0U) {
    const int sextet = sextetValue(text[size - (padding + 1U)]);
    const int mask = (padding == 2U) ? 0x0F : 0x03;
    if (sextet < 0 || (sextet & mask) != 0) {
      return {{}, Status::InvalidEncoding};
    }
  }

  std::vector<std::uint8_t> bytes((size / 4U) * 3U, 0U);
  const int decoded = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(bytes.data()),
    reinterpret_cast<const unsigned char *>(text.data()),
    static_cast<int>(size));
  if (decoded < 0) {
    LogEvent(kLogger, "base64_decode_failed").field("input_chars", size).error();
    return {{}, Status::InvalidEncoding};
  }

  // `EVP_DecodeBlock` materializes three bytes per quartet; trim the synthetic tail
  // bytes that correspond to validated '=' padding.
  bytes.resize(static_cast<std::size_t>(decoded) - padding);
  return {std::move(bytes), Status::Ok};
}

}  // namespace livekit_ros2_bridge::protocol::detail::base64
