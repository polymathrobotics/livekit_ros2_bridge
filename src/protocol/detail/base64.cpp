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

#include <string>

namespace livekit_ros2_bridge::protocol::detail::base64
{

namespace
{

int decodeSextet(char c) noexcept
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
  const int written = EVP_EncodeBlock(
    reinterpret_cast<unsigned char *>(text.data()),
    reinterpret_cast<const unsigned char *>(bytes),
    static_cast<int>(size));

  text.resize(static_cast<std::size_t>(written));
  return text;
}

Result decode(std::string_view text)
{
  if (text.empty()) {
    return {};
  }

  const std::size_t size = text.size();
  std::size_t padding = 0U;
  // Validate alphabet and terminal padding before length checks so missing padding has
  // a distinct status.
  for (const char c : text) {
    if (c == '=') {
      ++padding;
      continue;
    }

    if (padding != 0U || decodeSextet(c) < 0) {
      return {{}, Status::InvalidEncoding};
    }
  }

  if (padding > 2U || (padding != 0U && size == 1U)) {
    return {{}, Status::InvalidEncoding};
  }

  // After alphabet and padding checks, quartet misalignment is missing padding.
  if ((size % 4U) != 0U) {
    return {{}, Status::MissingPadding};
  }

  // RFC 4648 requires zero unused bits in the final quantum; OpenSSL accepts aliases.
  if (padding != 0U) {
    const int sextet = decodeSextet(text[size - (padding + 1U)]);
    const int mask = (padding == 2U) ? 0x0F : 0x03;
    if ((sextet & mask) != 0) {
      return {{}, Status::InvalidEncoding};
    }
  }

  std::vector<std::uint8_t> bytes((size / 4U) * 3U, 0U);
  const int written = EVP_DecodeBlock(
    reinterpret_cast<unsigned char *>(bytes.data()),
    reinterpret_cast<const unsigned char *>(text.data()),
    static_cast<int>(size));

  bytes.resize(static_cast<std::size_t>(written) - padding);
  return {std::move(bytes), Status::Ok};
}

}  // namespace livekit_ros2_bridge::protocol::detail::base64
