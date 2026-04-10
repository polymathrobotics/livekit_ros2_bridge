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

#include <errno.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <utility>

#include "gtest/gtest.h"
#include "utils/process_utils.hpp"

namespace livekit_ros2_bridge
{

constexpr int kChildExitCode = 23;

TEST(ProcessUtilsTest, UniqueFdMoveTransfersOwnership)
{
  int pipe_fds[2] = {-1, -1};
  ASSERT_EQ(pipe(pipe_fds), 0);

  UniqueFd original(pipe_fds[0]);
  UniqueFd moved(std::move(original));

  EXPECT_FALSE(original);
  EXPECT_TRUE(moved);
  EXPECT_EQ(moved.get(), pipe_fds[0]);

  const int released_fd = moved.release();
  EXPECT_EQ(released_fd, pipe_fds[0]);
  EXPECT_FALSE(moved);

  EXPECT_EQ(close(released_fd), 0);
  EXPECT_EQ(close(pipe_fds[1]), 0);
}

TEST(ProcessUtilsTest, UniqueFdResetAndDestructionCloseOwnedFd)
{
  int reset_pipe[2] = {-1, -1};
  ASSERT_EQ(pipe(reset_pipe), 0);

  UniqueFd fd(reset_pipe[0]);
  fd.reset();

  errno = 0;
  EXPECT_EQ(close(reset_pipe[0]), -1);
  EXPECT_EQ(errno, EBADF);
  EXPECT_EQ(close(reset_pipe[1]), 0);

  int destruction_pipe[2] = {-1, -1};
  ASSERT_EQ(pipe(destruction_pipe), 0);
  {
    UniqueFd destruction_fd(destruction_pipe[0]);
    EXPECT_TRUE(destruction_fd);
  }

  errno = 0;
  EXPECT_EQ(close(destruction_pipe[0]), -1);
  EXPECT_EQ(errno, EBADF);
  EXPECT_EQ(close(destruction_pipe[1]), 0);
}

TEST(ProcessUtilsTest, CreatePipePairAndReadWriteRoundTrip)
{
  PipePair pipe_pair;
  ASSERT_TRUE(createPipePair(pipe_pair));
  ASSERT_TRUE(pipe_pair.read_end);
  ASSERT_TRUE(pipe_pair.write_end);

  const char payload[] = "ok";
  ASSERT_EQ(writeNoIntr(pipe_pair.write_end.get(), payload, sizeof(payload)), static_cast<ssize_t>(sizeof(payload)));

  char buffer[sizeof(payload)] = {};
  ASSERT_EQ(readNoIntr(pipe_pair.read_end.get(), buffer, sizeof(buffer)), static_cast<ssize_t>(sizeof(buffer)));
  EXPECT_STREQ(buffer, payload);
}

TEST(ProcessUtilsTest, SetCloseOnExecSetsFdFlag)
{
  PipePair pipe_pair;
  ASSERT_TRUE(createPipePair(pipe_pair));
  ASSERT_TRUE(setCloseOnExec(pipe_pair.write_end.get()));

  const int flags = fcntl(pipe_pair.write_end.get(), F_GETFD);
  ASSERT_GE(flags, 0);
  EXPECT_NE(flags & FD_CLOEXEC, 0);
}

TEST(ProcessUtilsTest, WaitpidNoIntrReturnsChildExitStatus)
{
  const pid_t pid = fork();
  ASSERT_NE(pid, -1);

  if (pid == 0) {
    _exit(kChildExitCode);
  }

  int status = 0;
  ASSERT_EQ(waitpidNoIntr(pid, &status, 0), pid);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), kChildExitCode);
}

}  // namespace livekit_ros2_bridge
