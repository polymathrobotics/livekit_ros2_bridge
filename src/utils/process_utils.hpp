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

#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstddef>
#include <utility>

namespace livekit_ros2_bridge
{

class UniqueFd final
{
public:
  UniqueFd() = default;

  explicit UniqueFd(int fd) noexcept
  : fd_(fd)
  {}

  ~UniqueFd()
  {
    reset();
  }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : fd_(other.release())
  {}

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this == &other) {
      return *this;
    }

    reset(other.release());
    return *this;
  }

  int get() const noexcept
  {
    return fd_;
  }

  int release() noexcept
  {
    const int fd = fd_;
    fd_ = -1;
    return fd;
  }

  void reset(int fd = -1) noexcept
  {
    if (fd_ >= 0 && fd_ != fd) {
      (void)close(fd_);
    }
    fd_ = fd;
  }

  explicit operator bool() const noexcept
  {
    return fd_ >= 0;
  }

private:
  int fd_ = -1;
};

struct PipePair
{
  UniqueFd read_end;
  UniqueFd write_end;
};

inline bool createPipePair(PipePair & out) noexcept
{
  int pipe_fds[2] = {-1, -1};
  if (pipe(pipe_fds) != 0) {
    return false;
  }

  out.read_end.reset(pipe_fds[0]);
  out.write_end.reset(pipe_fds[1]);
  return true;
}

inline bool setCloseOnExec(int fd) noexcept
{
  const int flags = fcntl(fd, F_GETFD);
  if (flags < 0) {
    return false;
  }

  return fcntl(fd, F_SETFD, flags | FD_CLOEXEC) == 0;
}

inline pid_t waitpidNoIntr(pid_t pid, int * status, int options) noexcept
{
  pid_t result = -1;
  do {
    result = waitpid(pid, status, options);
  } while (result < 0 && errno == EINTR);
  return result;
}

inline ssize_t readNoIntr(int fd, void * buffer, std::size_t count) noexcept
{
  ssize_t result = -1;
  do {
    result = read(fd, buffer, count);
  } while (result < 0 && errno == EINTR);
  return result;
}

inline ssize_t writeNoIntr(int fd, const void * buffer, std::size_t count) noexcept
{
  ssize_t result = -1;
  do {
    result = write(fd, buffer, count);
  } while (result < 0 && errno == EINTR);
  return result;
}

}  // namespace livekit_ros2_bridge
