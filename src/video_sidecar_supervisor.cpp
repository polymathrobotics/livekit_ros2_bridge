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

#include "video_sidecar_supervisor.hpp"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#ifdef __linux__
  #include <sys/prctl.h>
#endif

#include <cmath>
#include <stdexcept>
#include <utility>

#include "access_policy.hpp"
#include "rclcpp/logging.hpp"
#include "utils/livekit_access_token.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("video_sidecar_supervisor");

void signalSidecarProcessGroup(pid_t pid, int signal_number)
{
  if (pid <= 0) {
    return;
  }

  if (kill(-pid, signal_number) == 0 || errno != ESRCH) {
    return;
  }

  // Fall back to the direct child if the process-group setup failed. The
  // normal path puts the sidecar in its own group so grandchildren are also
  // terminated during restart and shutdown.
  (void)kill(pid, signal_number);
}

VideoSidecarSupervisor::Config validateConfig(VideoSidecarSupervisor::Config config)
{
  if (config.livekit_url.empty()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires a non-empty livekit_url.");
  }
  if (config.livekit_room.empty()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires a non-empty livekit_room.");
  }
  if (config.bridge_identity.empty()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires a non-empty bridge_identity.");
  }
  if (config.api_key.empty()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires a non-empty api_key.");
  }
  if (config.api_secret.empty()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires a non-empty api_secret.");
  }
  if (config.token_ttl <= std::chrono::seconds::zero()) {
    throw std::invalid_argument("VideoSidecarSupervisor requires token_ttl > 0.");
  }
  if (config.unhealthy_restart_threshold == 0U) {
    throw std::invalid_argument("VideoSidecarSupervisor requires unhealthy_restart_threshold > 0.");
  }
  if (config.health_check_startup_grace < std::chrono::milliseconds::zero()) {
    config.health_check_startup_grace = std::chrono::milliseconds::zero();
  }
  return config;
}

}  // namespace

std::vector<std::string> buildGstreamerSidecarCommand(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token)
{
  std::vector<std::string> argv = {
    "gstreamer-publisher",
    "--url",
    livekit_url,
    "--token",
    livekit_token,
    "--",
  };
  argv.insert(argv.end(), spec.source_pipeline.begin(), spec.source_pipeline.end());
  return argv;
}

VideoSidecarSupervisor::VideoSidecarSupervisor(
  Config config, SidecarCommandBuilder build_sidecar_command, PublisherHealthCheck is_publisher_healthy)
: config_(validateConfig(std::move(config)))
, build_sidecar_command_(std::move(build_sidecar_command))
, is_publisher_healthy_(std::move(is_publisher_healthy))
{}

VideoSidecarSupervisor::~VideoSidecarSupervisor()
{
  shutdown();
}

std::string VideoSidecarSupervisor::ensureSidecar(const SidecarLaunchSpec & spec)
{
  if (spec.sidecar_key.empty()) {
    throw std::invalid_argument("Sidecar launch spec sidecar_key is required.");
  }
  if (is_shutdown_.load()) {
    throw std::runtime_error("Video sidecar supervisor is shut down.");
  }
  const std::string publisher_identity = derivePublisherIdentity(config_.bridge_identity, spec);

  reapExitedSidecars();

  auto [it, inserted] = ensureSidecarRecord(spec);
  it->second.publisher_identity = publisher_identity;
  if (it->second.pid <= 0) {
    try {
      restartSidecar(it->first, it->second);
    } catch (...) {
      if (inserted) {
        sidecars_.erase(it);
      }
      throw;
    }
  }
  return publisher_identity;
}

void VideoSidecarSupervisor::stopSidecar(const std::string & sidecar_key)
{
  auto it = sidecars_.find(sidecar_key);
  if (it == sidecars_.end()) {
    return;
  }
  killSidecar(it->first, it->second);
  sidecars_.erase(it);
}

bool VideoSidecarSupervisor::isSidecarRunning(const std::string & sidecar_key) const
{
  const auto it = sidecars_.find(sidecar_key);
  return it != sidecars_.end() && it->second.pid > 0;
}

void VideoSidecarSupervisor::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }
  for (auto & entry : sidecars_) {
    killSidecar(entry.first, entry.second);
  }
  sidecars_.clear();
}

std::string VideoSidecarSupervisor::derivePublisherIdentity(
  const std::string & bridge_identity, const SidecarLaunchSpec & spec)
{
  if (!spec.external_name.empty()) {
    return bridge_identity + "-video-source-" + keyToSlug(spec.external_name);
  }
  return bridge_identity + "-video-" + keyToSlug(spec.ros_topic);
}

std::string VideoSidecarSupervisor::keyToSlug(const std::string & key)
{
  std::string slug;
  slug.reserve(key.size());
  for (char ch : key) {
    if (ch == '/' || ch == ':') {
      if (!slug.empty() && slug.back() != '-') {
        slug.push_back('-');
      }
    } else {
      slug.push_back(ch);
    }
  }
  while (!slug.empty() && slug.back() == '-') {
    slug.pop_back();
  }
  return slug;
}

std::pair<VideoSidecarSupervisor::SidecarMap::iterator, bool> VideoSidecarSupervisor::ensureSidecarRecord(
  const SidecarLaunchSpec & spec)
{
  auto [it, inserted] = sidecars_.try_emplace(spec.sidecar_key);
  it->second.spec = spec;
  return {it, inserted};
}

VideoSidecarSupervisor::PreparedSidecarLaunch VideoSidecarSupervisor::prepareSidecarLaunch(
  const SidecarRecord & sidecar) const
{
  PreparedSidecarLaunch launch;
  launch.publisher_identity = derivePublisherIdentity(config_.bridge_identity, sidecar.spec);

  const auto now = std::chrono::system_clock::now();
  const std::string token = mintLiveKitAccessToken(
    config_.api_key,
    config_.api_secret,
    launch.publisher_identity,
    LiveKitRoomGrant{config_.livekit_room, true, true, false, false},
    now,
    config_.token_ttl);

  launch.argv = build_sidecar_command_(sidecar.spec, config_.livekit_url, token);
  if (launch.argv.empty()) {
    RCLCPP_ERROR(
      kLogger, "event=video_sidecar_command_error sidecar_key=%s reason=empty_argv", sidecar.spec.sidecar_key.c_str());
    throw std::runtime_error("Failed to build video sidecar command.");
  }

  launch.token_expires_at = now + config_.token_ttl;
  return launch;
}

void VideoSidecarSupervisor::restartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar)
{
  auto launch = prepareSidecarLaunch(sidecar);
  if (sidecar.pid > 0) {
    killSidecar(sidecar_key, sidecar);
  }
  spawnPreparedSidecar(sidecar_key, sidecar, std::move(launch));
}

void VideoSidecarSupervisor::tryRestartSidecar(
  const std::string & sidecar_key, SidecarRecord & sidecar, const char * reason)
{
  try {
    restartSidecar(sidecar_key, sidecar);
  } catch (const std::exception & exc) {
    const char * phase = sidecar.pid > 0 ? "prepare" : "spawn";
    RCLCPP_ERROR(
      kLogger,
      "event=video_sidecar_restart_failed sidecar_key=%s publisher_identity=%s phase=%s reason=%s error=%s",
      sidecar_key.c_str(),
      sidecar.publisher_identity.c_str(),
      phase,
      reason,
      exc.what());
  } catch (...) {
    const char * phase = sidecar.pid > 0 ? "prepare" : "spawn";
    RCLCPP_ERROR(
      kLogger,
      "event=video_sidecar_restart_failed sidecar_key=%s publisher_identity=%s phase=%s reason=%s",
      sidecar_key.c_str(),
      sidecar.publisher_identity.c_str(),
      phase,
      reason);
  }
}

void VideoSidecarSupervisor::spawnPreparedSidecar(
  const std::string & sidecar_key, SidecarRecord & sidecar, PreparedSidecarLaunch launch)
{
  int exec_status_pipe[2] = {-1, -1};
  if (pipe(exec_status_pipe) != 0) {
    RCLCPP_ERROR(kLogger, "pipe() failed for sidecar_key=%s: %s", sidecar_key.c_str(), strerror(errno));
    throw std::runtime_error("Failed to create video sidecar startup pipe.");
  }

  const int pipe_flags = fcntl(exec_status_pipe[1], F_GETFD);
  if (pipe_flags < 0 || fcntl(exec_status_pipe[1], F_SETFD, pipe_flags | FD_CLOEXEC) != 0) {
    const int error_code = errno;
    close(exec_status_pipe[0]);
    close(exec_status_pipe[1]);
    RCLCPP_ERROR(kLogger, "fcntl(FD_CLOEXEC) failed for sidecar_key=%s: %s", sidecar_key.c_str(), strerror(error_code));
    throw std::runtime_error("Failed to configure video sidecar startup pipe.");
  }

  const pid_t pid = fork();
  if (pid < 0) {
    close(exec_status_pipe[0]);
    close(exec_status_pipe[1]);
    RCLCPP_ERROR(kLogger, "fork() failed for sidecar_key=%s: %s", sidecar_key.c_str(), strerror(errno));
    throw std::runtime_error("Failed to fork video sidecar process.");
  }

  if (pid == 0) {
    close(exec_status_pipe[0]);
#ifdef __linux__
    prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif
    // Put each sidecar in its own process group so restart/shutdown can signal
    // the whole subtree. Some launchers are shell scripts that spawn helpers,
    // and killing only the direct child leaves grandchildren behind.
    (void)setpgid(0, 0);
    std::vector<char *> c_argv;
    c_argv.reserve(launch.argv.size() + 1);
    for (const auto & arg : launch.argv) {
      c_argv.push_back(const_cast<char *>(arg.c_str()));
    }
    c_argv.push_back(nullptr);

    execvp(c_argv[0], c_argv.data());

    const int exec_error = errno;
    ssize_t ignored = 0;
    do {
      ignored = write(exec_status_pipe[1], &exec_error, sizeof(exec_error));
    } while (ignored < 0 && errno == EINTR);
    close(exec_status_pipe[1]);
    _exit(127);
  }

  close(exec_status_pipe[1]);

  int exec_error = 0;
  ssize_t read_size = 0;
  do {
    read_size = read(exec_status_pipe[0], &exec_error, sizeof(exec_error));
  } while (read_size < 0 && errno == EINTR);
  const int read_error = errno;
  close(exec_status_pipe[0]);

  if (read_size < 0) {
    kill(pid, SIGKILL);
    int status = 0;
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    RCLCPP_ERROR(
      kLogger, "read() failed while waiting for sidecar_key=%s startup: %s", sidecar_key.c_str(), strerror(read_error));
    throw std::runtime_error("Failed waiting for video sidecar startup handshake.");
  }

  if (read_size != 0) {
    int status = 0;
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    if (read_size != static_cast<ssize_t>(sizeof(exec_error))) {
      RCLCPP_ERROR(
        kLogger,
        "startup handshake returned an invalid exec status for sidecar_key=%s identity=%s",
        sidecar_key.c_str(),
        launch.publisher_identity.c_str());
      throw std::runtime_error("Invalid video sidecar startup handshake response.");
    }
    RCLCPP_ERROR(
      kLogger,
      "execvp() failed for sidecar_key=%s identity=%s: %s",
      sidecar_key.c_str(),
      launch.publisher_identity.c_str(),
      strerror(exec_error));
    throw std::runtime_error("Failed to exec video sidecar process: " + std::string(strerror(exec_error)) + ".");
  }

  sidecar.pid = pid;
  sidecar.publisher_identity = launch.publisher_identity;
  sidecar.token_expires_at = launch.token_expires_at;
  sidecar.spawned_at = SteadyClock::now();
  sidecar.consecutive_unhealthy_checks = 0;
  RCLCPP_INFO(
    kLogger,
    "event=video_sidecar_spawned sidecar_key=%s publisher_identity=%s pid=%d",
    sidecar_key.c_str(),
    launch.publisher_identity.c_str(),
    static_cast<int>(pid));
}

void VideoSidecarSupervisor::killSidecar(const std::string & sidecar_key, SidecarRecord & sidecar)
{
  if (sidecar.pid <= 0) {
    return;
  }

  const pid_t pid = sidecar.pid;
  signalSidecarProcessGroup(pid, SIGTERM);

  int status = 0;
  pid_t result = waitpid(pid, &status, WNOHANG);
  if (result == 0) {
    usleep(500000);
    result = waitpid(pid, &status, WNOHANG);
    if (result == 0) {
      signalSidecarProcessGroup(pid, SIGKILL);
      waitpid(pid, &status, 0);
    }
  }

  RCLCPP_INFO(
    kLogger,
    "event=video_sidecar_stopped sidecar_key=%s publisher_identity=%s pid=%d",
    sidecar_key.c_str(),
    sidecar.publisher_identity.c_str(),
    static_cast<int>(pid));
  sidecar.pid = -1;
  sidecar.consecutive_unhealthy_checks = 0;
}

void VideoSidecarSupervisor::reapExitedSidecars()
{
  if (is_shutdown_.load()) {
    return;
  }
  for (auto & entry : sidecars_) {
    auto & sidecar = entry.second;
    if (sidecar.pid <= 0) {
      continue;
    }

    int status = 0;
    const pid_t result = waitpid(sidecar.pid, &status, WNOHANG);
    if (result == 0) {
      continue;
    }

    if (result < 0) {
      if (errno == ECHILD) {
        RCLCPP_WARN(
          kLogger,
          "event=video_sidecar_not_waitable sidecar_key=%s publisher_identity=%s pid=%d",
          entry.first.c_str(),
          sidecar.publisher_identity.c_str(),
          static_cast<int>(sidecar.pid));
        sidecar.pid = -1;
      } else {
        RCLCPP_ERROR(
          kLogger,
          "event=video_sidecar_waitpid_error sidecar_key=%s publisher_identity=%s pid=%d error=%s",
          entry.first.c_str(),
          sidecar.publisher_identity.c_str(),
          static_cast<int>(sidecar.pid),
          strerror(errno));
      }
      continue;
    }

    sidecar.pid = -1;

    if (WIFEXITED(status)) {
      RCLCPP_WARN(
        kLogger,
        "event=video_sidecar_exited sidecar_key=%s publisher_identity=%s exit_code=%d",
        entry.first.c_str(),
        sidecar.publisher_identity.c_str(),
        WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
      RCLCPP_WARN(
        kLogger,
        "event=video_sidecar_signaled sidecar_key=%s publisher_identity=%s signal=%d",
        entry.first.c_str(),
        sidecar.publisher_identity.c_str(),
        WTERMSIG(status));
    }
  }
}

void VideoSidecarSupervisor::restartUnhealthy()
{
  if (is_shutdown_.load() || !is_publisher_healthy_) {
    return;
  }

  const auto now = SteadyClock::now();
  for (auto & entry : sidecars_) {
    auto & sidecar = entry.second;
    if (sidecar.pid <= 0 || sidecar.publisher_identity.empty()) {
      continue;
    }

    // A freshly spawned sidecar needs time to join the room and publish its
    // video track before the bridge starts treating missing publications as a
    // failure that warrants a hard restart.
    if (now - sidecar.spawned_at < config_.health_check_startup_grace) {
      continue;
    }

    bool healthy = false;
    try {
      healthy = is_publisher_healthy_(sidecar.publisher_identity);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(
        kLogger,
        "event=video_sidecar_health_check_failed sidecar_key=%s publisher_identity=%s error=%s",
        entry.first.c_str(),
        sidecar.publisher_identity.c_str(),
        exc.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR(
        kLogger,
        "event=video_sidecar_health_check_failed sidecar_key=%s publisher_identity=%s",
        entry.first.c_str(),
        sidecar.publisher_identity.c_str());
      continue;
    }

    if (healthy) {
      sidecar.consecutive_unhealthy_checks = 0;
      continue;
    }

    ++sidecar.consecutive_unhealthy_checks;
    RCLCPP_WARN(
      kLogger,
      "event=video_sidecar_unhealthy sidecar_key=%s publisher_identity=%s miss_count=%zu threshold=%zu",
      entry.first.c_str(),
      sidecar.publisher_identity.c_str(),
      sidecar.consecutive_unhealthy_checks,
      config_.unhealthy_restart_threshold);
    if (sidecar.consecutive_unhealthy_checks < config_.unhealthy_restart_threshold) {
      continue;
    }

    RCLCPP_WARN(
      kLogger,
      "event=video_sidecar_restart sidecar_key=%s publisher_identity=%s reason=publisher_unhealthy",
      entry.first.c_str(),
      sidecar.publisher_identity.c_str());
    tryRestartSidecar(entry.first, sidecar, "publisher_unhealthy");
  }
}

void VideoSidecarSupervisor::restartExpiring()
{
  if (is_shutdown_.load()) {
    return;
  }
  const auto now = std::chrono::system_clock::now();
  const auto refresh_margin =
    std::chrono::duration_cast<std::chrono::system_clock::duration>(config_.token_refresh_margin);
  const auto half_ttl = std::chrono::duration_cast<std::chrono::system_clock::duration>(config_.token_ttl) / 2;
  const auto margin = std::min(refresh_margin, half_ttl);

  for (auto & entry : sidecars_) {
    auto & sidecar = entry.second;
    if (sidecar.pid <= 0) {
      continue;
    }

    const auto deadline = sidecar.token_expires_at - margin;
    if (now < deadline) {
      continue;
    }

    RCLCPP_INFO(
      kLogger,
      "event=video_sidecar_restart sidecar_key=%s publisher_identity=%s reason=token_expiring pid=%d",
      entry.first.c_str(),
      sidecar.publisher_identity.c_str(),
      static_cast<int>(sidecar.pid));

    tryRestartSidecar(entry.first, sidecar, "token_expiring");
  }
}

}  // namespace livekit_ros2_bridge
