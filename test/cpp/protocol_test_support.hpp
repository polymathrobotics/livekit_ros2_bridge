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

#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "gtest/gtest.h"
#include "protocol/validation_error.hpp"

namespace livekit_ros2_bridge::test_support
{

template <typename Fn>
void expectInvalidArgument(
  Fn && fn, std::string_view expected_message, std::optional<std::string_view> expected_field = std::nullopt)
{
  try {
    std::forward<Fn>(fn)();
    ADD_FAILURE() << "Expected std::invalid_argument";
    return;
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(std::string(error.what()), std::string(expected_message));
    if (!expected_field.has_value()) {
      return;
    }

    const auto * validation = dynamic_cast<const protocol::ValidationError *>(&error);
    ASSERT_NE(validation, nullptr);
    EXPECT_EQ(validation->field(), *expected_field);
  }
}

}  // namespace livekit_ros2_bridge::test_support
