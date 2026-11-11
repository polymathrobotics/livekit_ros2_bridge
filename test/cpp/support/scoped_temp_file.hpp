// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

#include "gtest/gtest.h"

namespace livekit_ros2_bridge::test_support
{

// A uniquely named file in the test temp directory, removed on destruction.
class ScopedTempFile final
{
public:
  explicit ScopedTempFile(const std::string & prefix)
  {
    std::string path = ::testing::TempDir() + prefix + "XXXXXX";
    const int descriptor = ::mkstemp(path.data());
    if (descriptor < 0) {
      throw std::runtime_error("Failed to create a temp file for " + prefix);
    }
    ::close(descriptor);
    path_ = std::move(path);
  }

  ~ScopedTempFile()
  {
    remove();
  }

  ScopedTempFile(const ScopedTempFile &) = delete;
  ScopedTempFile & operator=(const ScopedTempFile &) = delete;
  ScopedTempFile(ScopedTempFile &&) = delete;
  ScopedTempFile & operator=(ScopedTempFile &&) = delete;

  const std::string & path() const
  {
    return path_;
  }

  // Safe to call more than once.
  void remove() const
  {
    (void)std::remove(path_.c_str());
  }

private:
  std::string path_;
};

}  // namespace livekit_ros2_bridge::test_support
