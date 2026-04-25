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

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace livekit_ros2_bridge::protocol
{

class ValidationError final : public std::invalid_argument
{
public:
  ValidationError(std::string field, std::string reason)
  : std::invalid_argument(reason)
  , field_(std::move(field))
  {}

  /// The field view is valid for the exception lifetime.
  std::string_view field() const noexcept
  {
    return field_;
  }

private:
  std::string field_;
};

}  // namespace livekit_ros2_bridge::protocol
