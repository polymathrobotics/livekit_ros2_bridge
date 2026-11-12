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

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <mutex>
#include <string>
#include <vector>

#include "rcutils/logging.h"

namespace livekit_ros2_bridge::test_support
{

// Records formatted log lines (still forwarding them) so tests can assert on log events.
class ScopedLogCapture final
{
public:
  ScopedLogCapture()
  {
    std::lock_guard<std::mutex> lock(mutex());
    lines().clear();
    previousHandler() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&ScopedLogCapture::handle);
  }

  ~ScopedLogCapture()
  {
    std::lock_guard<std::mutex> lock(mutex());
    rcutils_logging_set_output_handler(previousHandler());
  }

  ScopedLogCapture(const ScopedLogCapture &) = delete;
  ScopedLogCapture & operator=(const ScopedLogCapture &) = delete;
  ScopedLogCapture(ScopedLogCapture &&) = delete;
  ScopedLogCapture & operator=(ScopedLogCapture &&) = delete;

  // Returns how many captured lines contain every fragment.
  std::size_t countLines(std::initializer_list<const char *> fragments) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::size_t count = 0U;
    for (const auto & line : lines()) {
      if (containsAll(line, fragments)) {
        ++count;
      }
    }
    return count;
  }

  // Returns, in logged order, the value of the last ` key=` field in each captured line that
  // contains every fragment: the text up to the next space, or empty when the line lacks it.
  std::vector<std::string> fieldValues(std::initializer_list<const char *> fragments, const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    const std::string field_prefix = " " + key + "=";
    std::vector<std::string> values;
    for (const auto & line : lines()) {
      if (!containsAll(line, fragments)) {
        continue;
      }
      const std::size_t field_start = line.rfind(field_prefix);
      if (field_start == std::string::npos) {
        values.emplace_back();
        continue;
      }
      const std::size_t value_start = field_start + field_prefix.size();
      values.push_back(line.substr(value_start, line.find(' ', value_start) - value_start));
    }
    return values;
  }

private:
  static bool containsAll(const std::string & line, std::initializer_list<const char *> fragments)
  {
    for (const char * fragment : fragments) {
      if (line.find(fragment) == std::string::npos) {
        return false;
      }
    }
    return true;
  }

  static void handle(
    const rcutils_log_location_t * location,
    int severity,
    const char * name,
    rcutils_time_point_value_t timestamp,
    const char * format,
    va_list * args)
  {
    va_list arguments;
    va_copy(arguments, *args);
    char buffer[4096];
    std::vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    rcutils_logging_output_handler_t previous_handler = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex());
      lines().emplace_back(buffer);
      previous_handler = previousHandler();
    }
    if (previous_handler != nullptr) {
      previous_handler(location, severity, name, timestamp, format, args);
    }
  }

  static std::mutex & mutex()
  {
    static std::mutex instance;
    return instance;
  }

  static std::vector<std::string> & lines()
  {
    static std::vector<std::string> instance;
    return instance;
  }

  static rcutils_logging_output_handler_t & previousHandler()
  {
    static rcutils_logging_output_handler_t instance = nullptr;
    return instance;
  }
};

}  // namespace livekit_ros2_bridge::test_support
