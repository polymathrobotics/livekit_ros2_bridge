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

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "video_config.hpp"
#include "video_sidecar_supervisor.hpp"

namespace livekit_ros2_bridge
{
namespace
{

bool waitUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

VideoSidecarSupervisor::Config makeTestConfig()
{
  VideoSidecarSupervisor::Config config;
  config.livekit_url = "ws://localhost:7880";
  config.livekit_room = "test-room";
  config.api_key = "test-api-key";
  config.api_secret = "test-api-secret";
  config.token_ttl = std::chrono::seconds(600);
  config.bridge_identity = "bridge-test";
  return config;
}

SidecarLaunchSpec makeRosSpec(const std::string & topic, const std::string & interface_type)
{
  VideoConfig config = makeDefaultVideoConfig();
  return resolveRosVideoLaunchSpec(config, topic, interface_type);
}

SidecarLaunchSpec makeConfiguredSpec(const std::string & external_name)
{
  VideoConfig config = makeDefaultVideoConfig();
  config.pipeline_sources.emplace(
    normalizeExternalName(external_name),
    ConfiguredPipelineSource{
      "uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0 ! videoconvert ! vp8enc deadline=1"});
  return resolvePipelineVideoLaunchSpec(config, external_name);
}

std::vector<std::string> sleepSidecarCommandBuilder(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token)
{
  (void)spec;
  (void)livekit_url;
  (void)livekit_token;
  return {"sleep", "3600"};
}

SidecarCommandBuilder makeCountingBuilder(int & spawn_count)
{
  return
    [&spawn_count](const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
      ++spawn_count;
      return {"sleep", "3600"};
    };
}

void expectInvalidConfig(VideoSidecarSupervisor::Config config, const char * expected_error)
{
  try {
    (void)VideoSidecarSupervisor(std::move(config), sleepSidecarCommandBuilder);
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), expected_error);
  }
}

void expectGstreamerSidecarCommandPrefix(
  const std::vector<std::string> & cmd, const std::string & livekit_url, const std::string & livekit_token)
{
  ASSERT_GE(cmd.size(), 6U);
  EXPECT_EQ(cmd[0], "gstreamer-publisher");
  EXPECT_EQ(cmd[1], "--url");
  EXPECT_EQ(cmd[2], livekit_url);
  EXPECT_EQ(cmd[3], "--token");
  EXPECT_EQ(cmd[4], livekit_token);
  EXPECT_EQ(cmd[5], "--");
}

TEST(VideoSidecarSupervisorTest, ConstructorRejectsInvalidConfig)
{
  struct InvalidConfigCase
  {
    const char * name;
    std::function<void(VideoSidecarSupervisor::Config &)> mutate;
    const char * error;
  };

  const std::vector<InvalidConfigCase> cases = {
    {"empty livekit_url",
     [](VideoSidecarSupervisor::Config & config) { config.livekit_url.clear(); },
     "VideoSidecarSupervisor requires a non-empty livekit_url."},
    {"empty livekit_room",
     [](VideoSidecarSupervisor::Config & config) { config.livekit_room.clear(); },
     "VideoSidecarSupervisor requires a non-empty livekit_room."},
    {"empty bridge_identity",
     [](VideoSidecarSupervisor::Config & config) { config.bridge_identity.clear(); },
     "VideoSidecarSupervisor requires a non-empty bridge_identity."},
    {"empty api_key",
     [](VideoSidecarSupervisor::Config & config) { config.api_key.clear(); },
     "VideoSidecarSupervisor requires a non-empty api_key."},
    {"empty api_secret",
     [](VideoSidecarSupervisor::Config & config) { config.api_secret.clear(); },
     "VideoSidecarSupervisor requires a non-empty api_secret."},
    {"non-positive token_ttl",
     [](VideoSidecarSupervisor::Config & config) { config.token_ttl = std::chrono::seconds::zero(); },
     "VideoSidecarSupervisor requires token_ttl > 0."},
    {"zero unhealthy_restart_threshold",
     [](VideoSidecarSupervisor::Config & config) { config.unhealthy_restart_threshold = 0U; },
     "VideoSidecarSupervisor requires unhealthy_restart_threshold > 0."},
  };

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    auto config = makeTestConfig();
    test_case.mutate(config);
    expectInvalidConfig(std::move(config), test_case.error);
  }
}

TEST(VideoSidecarSupervisorTest, EnsureSidecarSpawnsOnceAndReusesRunningSidecar)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  const std::string first = supervisor.ensureSidecar(spec);
  const std::string second = supervisor.ensureSidecar(spec);

  EXPECT_EQ(first, "bridge-test-video-camera-front");
  EXPECT_EQ(first, second);
  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  EXPECT_EQ(spawn_count, 1);

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, EnsureSidecarRespawnsStoppedTopicAndKeepsIdentity)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));

  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");
  const std::string first = supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));

  supervisor.stopSidecar(spec.sidecar_key);
  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));

  const std::string second = supervisor.ensureSidecar(spec);

  EXPECT_EQ(first, second);
  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  EXPECT_EQ(spawn_count, 2);

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, ConfiguredSourceUsesCanonicalSidecarKeyForStopAndRestart)
{
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), sleepSidecarCommandBuilder);
  const auto configured_spec = makeConfiguredSpec("/sources/front");
  const std::string first = supervisor.ensureSidecar(configured_spec);

  ASSERT_TRUE(supervisor.isSidecarRunning(configured_spec.sidecar_key));

  supervisor.stopSidecar(configured_spec.sidecar_key);
  ASSERT_FALSE(supervisor.isSidecarRunning(configured_spec.sidecar_key));

  const std::string second = supervisor.ensureSidecar(configured_spec);

  EXPECT_EQ(first, "bridge-test-video-source-sources-front");
  EXPECT_EQ(second, first);
  EXPECT_TRUE(supervisor.isSidecarRunning(configured_spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, RejectsInvalidSidecarLaunchSpecSidecarKey)
{
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), sleepSidecarCommandBuilder);

  SidecarLaunchSpec spec;
  try {
    (void)supervisor.ensureSidecar(spec);
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "Sidecar launch spec sidecar_key is required.");
  }
}

TEST(VideoSidecarSupervisorTest, EnsureSidecarThrowsWhenSidecarCommandBuilderReturnsEmptyArgv)
{
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(
    std::move(config),
    [](const SidecarLaunchSpec &, const std::string &, const std::string &) { return std::vector<std::string>{}; });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  EXPECT_THROW(supervisor.ensureSidecar(spec), std::runtime_error);
  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, EnsureSidecarThrowsWhenExecFailsAfterFork)
{
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(
    std::move(config), [](const SidecarLaunchSpec &, const std::string &, const std::string &) {
      return std::vector<std::string>{"/definitely/missing/gstreamer-publisher"};
    });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  EXPECT_THROW(supervisor.ensureSidecar(spec), std::runtime_error);
  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, DistinctSpecsSpawnDistinctSidecarsAndShutdownStopsThem)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));
  const auto front_spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");
  const auto rear_spec = makeRosSpec("/camera/rear", "sensor_msgs/msg/Image");

  const std::string front_identity = supervisor.ensureSidecar(front_spec);
  const std::string rear_identity = supervisor.ensureSidecar(rear_spec);

  EXPECT_NE(front_identity, rear_identity);
  EXPECT_TRUE(supervisor.isSidecarRunning(front_spec.sidecar_key));
  EXPECT_TRUE(supervisor.isSidecarRunning(rear_spec.sidecar_key));
  EXPECT_EQ(spawn_count, 2);

  supervisor.shutdown();

  EXPECT_FALSE(supervisor.isSidecarRunning(front_spec.sidecar_key));
  EXPECT_FALSE(supervisor.isSidecarRunning(rear_spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, ShutdownKillsShellScriptGrandchildren)
{
  const auto temp_dir = std::filesystem::temp_directory_path() / "video_sidecar_supervisor_process_group_test";
  std::filesystem::remove_all(temp_dir);
  ASSERT_TRUE(std::filesystem::create_directories(temp_dir));

  const auto script_path = temp_dir / "fake_sidecar.sh";
  const auto child_pid_path = temp_dir / "child.pid";
  {
    std::ofstream script(script_path);
    ASSERT_TRUE(script.is_open());
    script << "#!/bin/sh\n";
    script << "sleep 3600 &\n";
    script << "echo $! > '" << child_pid_path.string() << "'\n";
    script << "wait $!\n";
  }
  std::filesystem::permissions(
    script_path,
    std::filesystem::perms::owner_exec | std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
    std::filesystem::perm_options::replace);

  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(
    std::move(config), [script_path](const SidecarLaunchSpec &, const std::string &, const std::string &) {
      return std::vector<std::string>{script_path.string()};
    });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);

  ASSERT_TRUE(waitUntil([&child_pid_path]() { return std::filesystem::exists(child_pid_path); }));

  pid_t grandchild_pid = -1;
  ASSERT_TRUE(waitUntil([&child_pid_path, &grandchild_pid]() {
    std::ifstream child_pid_stream(child_pid_path);
    if (!child_pid_stream.is_open()) {
      return false;
    }

    pid_t parsed_pid = -1;
    child_pid_stream >> parsed_pid;
    if (parsed_pid <= 0) {
      return false;
    }

    grandchild_pid = parsed_pid;
    return true;
  }));
  ASSERT_GT(grandchild_pid, 0);
  ASSERT_EQ(kill(grandchild_pid, 0), 0);

  supervisor.shutdown();

  EXPECT_TRUE(waitUntil([grandchild_pid]() { return kill(grandchild_pid, 0) != 0; }));
  std::filesystem::remove_all(temp_dir);
}

TEST(VideoSidecarSupervisorTest, EnsureSidecarRejectsRespawnAfterShutdown)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_EQ(spawn_count, 1);

  supervisor.shutdown();

  EXPECT_THROW(supervisor.ensureSidecar(spec), std::runtime_error);
  EXPECT_EQ(spawn_count, 1);
  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, RestartExpiringRestartsSidecarWithExpiredToken)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  config.token_ttl = std::chrono::seconds(1);
  config.token_refresh_margin = std::chrono::seconds(1);
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  ASSERT_EQ(spawn_count, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(700));

  supervisor.restartExpiring();

  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  EXPECT_EQ(spawn_count, 2);

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, RestartExpiringDoesNotRespawnAfterShutdown)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  config.token_ttl = std::chrono::seconds(1);
  config.token_refresh_margin = std::chrono::seconds(1);
  VideoSidecarSupervisor supervisor(std::move(config), makeCountingBuilder(spawn_count));
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_EQ(spawn_count, 1);

  supervisor.shutdown();
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  supervisor.restartExpiring();

  EXPECT_EQ(spawn_count, 1);
  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));
}

TEST(VideoSidecarSupervisorTest, RestartUnhealthyRespawnsAfterConfiguredMissThreshold)
{
  int spawn_count = 0;
  bool healthy = true;
  auto config = makeTestConfig();
  config.health_check_startup_grace = std::chrono::milliseconds::zero();
  config.unhealthy_restart_threshold = 2U;
  VideoSidecarSupervisor supervisor(
    std::move(config), makeCountingBuilder(spawn_count), [&healthy](const std::string &) { return healthy; });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  ASSERT_EQ(spawn_count, 1);

  healthy = false;
  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 1);

  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 2);
  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, RestartUnhealthyHonorsStartupGrace)
{
  int spawn_count = 0;
  auto config = makeTestConfig();
  config.health_check_startup_grace = std::chrono::hours(1);
  config.unhealthy_restart_threshold = 1U;
  VideoSidecarSupervisor supervisor(
    std::move(config), makeCountingBuilder(spawn_count), [](const std::string &) { return false; });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));

  supervisor.restartUnhealthy();

  EXPECT_EQ(spawn_count, 1);
  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, RestartUnhealthyResetsMissCountAfterHealthyCheck)
{
  int spawn_count = 0;
  bool healthy = true;
  auto config = makeTestConfig();
  config.health_check_startup_grace = std::chrono::milliseconds::zero();
  config.unhealthy_restart_threshold = 2U;
  VideoSidecarSupervisor supervisor(
    std::move(config), makeCountingBuilder(spawn_count), [&healthy](const std::string &) { return healthy; });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));

  healthy = false;
  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 1);

  healthy = true;
  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 1);

  healthy = false;
  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 1);

  supervisor.restartUnhealthy();
  EXPECT_EQ(spawn_count, 2);

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, RestartExpiringKeepsRunningSidecarWhenReplacementPreparationFails)
{
  int spawn_count = 0;
  bool fail_refresh = false;
  auto flaky_builder =
    [&spawn_count, &fail_refresh](
      const SidecarLaunchSpec &, const std::string &, const std::string &) -> std::vector<std::string> {
    ++spawn_count;
    if (fail_refresh) {
      return {};
    }
    return {"sleep", "3600"};
  };

  auto config = makeTestConfig();
  config.token_ttl = std::chrono::seconds(1);
  config.token_refresh_margin = std::chrono::seconds(1);
  VideoSidecarSupervisor supervisor(std::move(config), flaky_builder);
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);
  ASSERT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  ASSERT_EQ(spawn_count, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(700));

  fail_refresh = true;
  EXPECT_NO_THROW(supervisor.restartExpiring());

  EXPECT_TRUE(supervisor.isSidecarRunning(spec.sidecar_key));
  EXPECT_EQ(spawn_count, 2);

  supervisor.shutdown();
}

TEST(VideoSidecarSupervisorTest, ReapChildrenLeavesUnmanagedChildWaitable)
{
  auto config = makeTestConfig();
  VideoSidecarSupervisor supervisor(
    std::move(config), [](const SidecarLaunchSpec &, const std::string &, const std::string &) {
      return std::vector<std::string>{"/bin/sh", "-c", "exit 0"};
    });
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");

  supervisor.ensureSidecar(spec);

  const pid_t unrelated_pid = fork();
  ASSERT_NE(unrelated_pid, -1);
  if (unrelated_pid == 0) {
    _exit(23);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  supervisor.reapExitedSidecars();

  EXPECT_FALSE(supervisor.isSidecarRunning(spec.sidecar_key));

  int unrelated_status = 0;
  ASSERT_EQ(waitpid(unrelated_pid, &unrelated_status, 0), unrelated_pid);
  EXPECT_TRUE(WIFEXITED(unrelated_status));
  EXPECT_EQ(WEXITSTATUS(unrelated_status), 23);

  supervisor.shutdown();
}

TEST(BuildGstreamerSidecarCommandTest, RosImagePassesPipelineThroughUnmodified)
{
  const auto spec = makeRosSpec("/camera/front", "sensor_msgs/msg/Image");
  const auto cmd = buildGstreamerSidecarCommand(spec, "wss://lk.example.com", "test-token-123");

  expectGstreamerSidecarCommandPrefix(cmd, "wss://lk.example.com", "test-token-123");
  ASSERT_EQ(cmd.size(), spec.source_pipeline.size() + 6U);
  EXPECT_EQ(std::vector<std::string>(cmd.begin() + 6, cmd.end()), spec.source_pipeline);
}

TEST(BuildGstreamerSidecarCommandTest, PipelineSourceWithBangSeparators)
{
  VideoConfig config = makeDefaultVideoConfig();
  config.pipeline_sources.emplace(
    "/sources/test", ConfiguredPipelineSource{"videotestsrc pattern=ball ! video/x-raw,width=640,height=480 ! vp8enc"});
  const auto spec = resolvePipelineVideoLaunchSpec(config, "/sources/test");

  const auto cmd = buildGstreamerSidecarCommand(spec, "wss://lk.example.com", "t");

  expectGstreamerSidecarCommandPrefix(cmd, "wss://lk.example.com", "t");
  ASSERT_EQ(cmd.size(), spec.source_pipeline.size() + 6U);
  EXPECT_EQ(std::vector<std::string>(cmd.begin() + 6, cmd.end()), spec.source_pipeline);
}

}  // namespace
}  // namespace livekit_ros2_bridge
