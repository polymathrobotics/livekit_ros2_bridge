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
#include "utils/log_event.hpp"
#include "utils/process_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("video_sidecar_supervisor");
constexpr auto kUnhealthyLogThrottlePeriod = std::chrono::seconds(5);

void logSidecarSubprocessFailure(
  const std::string & sidecar_key,
  const std::string & publisher_identity,
  const char * phase,
  const char * reason,
  const char * error,
  pid_t pid)
{
  LogEvent(kLogger, "video_sidecar_subprocess_failed")
    .kv("sidecar_key", sidecar_key)
    .kv("publisher_identity", publisher_identity)
    .kv("phase", phase)
    .kv("reason", reason)
    .kv("error", error)
    .kv("pid", static_cast<int>(pid))
    .error();
}

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
  reapExitedSidecars();

  auto [it, inserted] = sidecars_.try_emplace(spec.sidecar_key);
  it->second.spec = spec;
  if (it->second.publisher_identity.empty()) {
    it->second.publisher_identity = derivePublisherIdentity(config_.bridge_identity, spec);
  }
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
  return it->second.publisher_identity;
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

VideoSidecarSupervisor::PreparedSidecarLaunch VideoSidecarSupervisor::prepareSidecarLaunch(
  const SidecarRecord & sidecar) const
{
  PreparedSidecarLaunch launch;
  const auto now = std::chrono::system_clock::now();
  const std::string token = mintLiveKitAccessToken(
    config_.api_key,
    config_.api_secret,
    sidecar.publisher_identity,
    LiveKitRoomGrant{config_.livekit_room, true, true, false, false},
    now,
    config_.token_ttl);

  launch.argv = build_sidecar_command_(sidecar.spec, config_.livekit_url, token);
  if (launch.argv.empty()) {
    logSidecarSubprocessFailure(
      sidecar.spec.sidecar_key,
      sidecar.publisher_identity,
      "prepare",
      "empty_argv",
      "command_builder_returned_empty_argv",
      -1);
    throw std::runtime_error("Failed to build video sidecar command.");
  }

  launch.token_expires_at = now + config_.token_ttl;
  return launch;
}

void VideoSidecarSupervisor::restartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar)
{
  // Prepare the replacement before killing the current child so command or token
  // failures leave the existing publisher running.
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
    LogEvent(kLogger, "video_sidecar_restart_failed")
      .kv("sidecar_key", sidecar_key)
      .kv("publisher_identity", sidecar.publisher_identity)
      .kv("phase", phase)
      .kv("reason", reason)
      .kv("error", exc.what())
      .error();
  } catch (...) {
    const char * phase = sidecar.pid > 0 ? "prepare" : "spawn";
    LogEvent(kLogger, "video_sidecar_restart_failed")
      .kv("sidecar_key", sidecar_key)
      .kv("publisher_identity", sidecar.publisher_identity)
      .kv("phase", phase)
      .kv("reason", reason)
      .error();
  }
}

void VideoSidecarSupervisor::spawnPreparedSidecar(
  const std::string & sidecar_key, SidecarRecord & sidecar, PreparedSidecarLaunch launch)
{
  // The child writes errno only when execvp() fails. On success, FD_CLOEXEC
  // closes the write end during exec and the parent observes EOF instead.
  PipePair exec_status_pipe;
  if (!createPipePair(exec_status_pipe)) {
    const int error_code = errno;
    logSidecarSubprocessFailure(
      sidecar_key, sidecar.publisher_identity, "spawn", "pipe_failed", strerror(error_code), -1);
    throw std::runtime_error("Failed to create video sidecar startup pipe.");
  }

  if (!setCloseOnExec(exec_status_pipe.write_end.get())) {
    const int error_code = errno;
    logSidecarSubprocessFailure(
      sidecar_key, sidecar.publisher_identity, "spawn", "startup_pipe_cloexec_failed", strerror(error_code), -1);
    throw std::runtime_error("Failed to configure video sidecar startup pipe.");
  }

  const pid_t pid = fork();
  if (pid < 0) {
    const int error_code = errno;
    logSidecarSubprocessFailure(
      sidecar_key, sidecar.publisher_identity, "spawn", "fork_failed", strerror(error_code), -1);
    throw std::runtime_error("Failed to fork video sidecar process.");
  }

  if (pid == 0) {
    exec_status_pipe.read_end.reset();
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
    const ssize_t ignored = writeNoIntr(exec_status_pipe.write_end.get(), &exec_error, sizeof(exec_error));
    (void)ignored;
    exec_status_pipe.write_end.reset();
    _exit(127);
  }

  exec_status_pipe.write_end.reset();

  int exec_error = 0;
  const ssize_t read_size = readNoIntr(exec_status_pipe.read_end.get(), &exec_error, sizeof(exec_error));
  const int read_error = errno;
  exec_status_pipe.read_end.reset();

  if (read_size < 0) {
    kill(pid, SIGKILL);
    int status = 0;
    (void)waitpidNoIntr(pid, &status, 0);
    logSidecarSubprocessFailure(
      sidecar_key, sidecar.publisher_identity, "spawn", "startup_handshake_read_failed", strerror(read_error), pid);
    throw std::runtime_error("Failed waiting for video sidecar startup handshake.");
  }

  if (read_size != 0) {
    int status = 0;
    (void)waitpidNoIntr(pid, &status, 0);
    if (read_size != static_cast<ssize_t>(sizeof(exec_error))) {
      logSidecarSubprocessFailure(
        sidecar_key,
        sidecar.publisher_identity,
        "spawn",
        "startup_handshake_invalid",
        "unexpected_exec_status_size",
        pid);
      throw std::runtime_error("Invalid video sidecar startup handshake response.");
    }
    logSidecarSubprocessFailure(
      sidecar_key, sidecar.publisher_identity, "spawn", "exec_failed", strerror(exec_error), pid);
    throw std::runtime_error("Failed to exec video sidecar process: " + std::string(strerror(exec_error)) + ".");
  }

  sidecar.pid = pid;
  sidecar.token_expires_at = launch.token_expires_at;
  sidecar.spawned_at = SteadyClock::now();
  sidecar.consecutive_unhealthy_checks = 0;
  sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
  LogEvent(kLogger, "video_sidecar_spawned")
    .kv("sidecar_key", sidecar_key)
    .kv("publisher_identity", sidecar.publisher_identity)
    .kv("pid", static_cast<int>(pid))
    .info();
}

void VideoSidecarSupervisor::killSidecar(const std::string & sidecar_key, SidecarRecord & sidecar)
{
  if (sidecar.pid <= 0) {
    return;
  }

  const pid_t pid = sidecar.pid;
  signalSidecarProcessGroup(pid, SIGTERM);

  int status = 0;
  pid_t result = waitpidNoIntr(pid, &status, WNOHANG);
  if (result < 0) {
    const int error_code = errno;
    LogEvent(kLogger, "video_sidecar_stop_failed")
      .kv("sidecar_key", sidecar_key)
      .kv("publisher_identity", sidecar.publisher_identity)
      .kv("phase", "wait")
      .kv("reason", "sigterm_waitpid_failed")
      .kv("error", strerror(error_code))
      .kv("pid", static_cast<int>(pid))
      .error();
    sidecar.pid = -1;
    sidecar.consecutive_unhealthy_checks = 0;
    sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
    return;
  }

  if (result == 0) {
    usleep(500000);
    result = waitpidNoIntr(pid, &status, WNOHANG);
    if (result < 0) {
      const int error_code = errno;
      LogEvent(kLogger, "video_sidecar_stop_failed")
        .kv("sidecar_key", sidecar_key)
        .kv("publisher_identity", sidecar.publisher_identity)
        .kv("phase", "wait")
        .kv("reason", "sigterm_waitpid_failed")
        .kv("error", strerror(error_code))
        .kv("pid", static_cast<int>(pid))
        .error();
      sidecar.pid = -1;
      sidecar.consecutive_unhealthy_checks = 0;
      sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
      return;
    }

    if (result == 0) {
      signalSidecarProcessGroup(pid, SIGKILL);
      result = waitpidNoIntr(pid, &status, 0);
      if (result < 0) {
        const int error_code = errno;
        LogEvent(kLogger, "video_sidecar_stop_failed")
          .kv("sidecar_key", sidecar_key)
          .kv("publisher_identity", sidecar.publisher_identity)
          .kv("phase", "wait")
          .kv("reason", "sigkill_waitpid_failed")
          .kv("error", strerror(error_code))
          .kv("pid", static_cast<int>(pid))
          .error();
        sidecar.pid = -1;
        sidecar.consecutive_unhealthy_checks = 0;
        sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
        return;
      }

      LogEvent(kLogger, "video_sidecar_killed")
        .kv("sidecar_key", sidecar_key)
        .kv("publisher_identity", sidecar.publisher_identity)
        .kv("reason", "sigterm_timeout")
        .kv("pid", static_cast<int>(pid))
        .warn();
      sidecar.pid = -1;
      sidecar.consecutive_unhealthy_checks = 0;
      sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
      return;
    }
  }

  LogEvent(kLogger, "video_sidecar_stopped")
    .kv("sidecar_key", sidecar_key)
    .kv("publisher_identity", sidecar.publisher_identity)
    .kv("pid", static_cast<int>(pid))
    .info();
  sidecar.pid = -1;
  sidecar.consecutive_unhealthy_checks = 0;
  sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
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
    // Reap only known sidecar pids so unrelated children remain waitable by their owner.
    const pid_t result = waitpidNoIntr(sidecar.pid, &status, WNOHANG);
    if (result == 0) {
      continue;
    }

    if (result < 0) {
      if (errno == ECHILD) {
        LogEvent(kLogger, "video_sidecar_not_waitable")
          .kv("sidecar_key", entry.first)
          .kv("publisher_identity", sidecar.publisher_identity)
          .kv("pid", static_cast<int>(sidecar.pid))
          .warn();
        sidecar.pid = -1;
      } else {
        LogEvent(kLogger, "video_sidecar_waitpid_error")
          .kv("sidecar_key", entry.first)
          .kv("publisher_identity", sidecar.publisher_identity)
          .kv("pid", static_cast<int>(sidecar.pid))
          .kv("error", strerror(errno))
          .error();
      }
      continue;
    }

    sidecar.pid = -1;

    if (WIFEXITED(status)) {
      LogEvent(kLogger, "video_sidecar_exited")
        .kv("sidecar_key", entry.first)
        .kv("publisher_identity", sidecar.publisher_identity)
        .kv("exit_code", WEXITSTATUS(status))
        .warn();
    } else if (WIFSIGNALED(status)) {
      LogEvent(kLogger, "video_sidecar_signaled")
        .kv("sidecar_key", entry.first)
        .kv("publisher_identity", sidecar.publisher_identity)
        .kv("signal", WTERMSIG(status))
        .warn();
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
      LogEvent(kLogger, "video_sidecar_health_check_failed")
        .kv("sidecar_key", entry.first)
        .kv("publisher_identity", sidecar.publisher_identity)
        .kv("error", exc.what())
        .error();
      continue;
    } catch (...) {
      LogEvent(kLogger, "video_sidecar_health_check_failed")
        .kv("sidecar_key", entry.first)
        .kv("publisher_identity", sidecar.publisher_identity)
        .error();
      continue;
    }

    if (healthy) {
      sidecar.consecutive_unhealthy_checks = 0;
      sidecar.next_unhealthy_log_at = SteadyClock::time_point{};
      continue;
    }

    ++sidecar.consecutive_unhealthy_checks;
    if (sidecar.consecutive_unhealthy_checks < config_.unhealthy_restart_threshold) {
      if (sidecar.next_unhealthy_log_at == SteadyClock::time_point{} || now >= sidecar.next_unhealthy_log_at) {
        LogEvent(kLogger, "video_sidecar_unhealthy")
          .kv("sidecar_key", entry.first)
          .kv("publisher_identity", sidecar.publisher_identity)
          .kv("reason", "publisher_unhealthy")
          .kv("count", sidecar.consecutive_unhealthy_checks)
          .kv("threshold", config_.unhealthy_restart_threshold)
          .warn();
        sidecar.next_unhealthy_log_at = now + kUnhealthyLogThrottlePeriod;
      }
      continue;
    }

    LogEvent(kLogger, "video_sidecar_restart")
      .kv("sidecar_key", entry.first)
      .kv("publisher_identity", sidecar.publisher_identity)
      .kv("reason", "publisher_unhealthy")
      .warn();
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
  // Clamp the lead time so an oversized configured margin does not make every fresh sidecar immediately expire.
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

    LogEvent(kLogger, "video_sidecar_restart")
      .kv("sidecar_key", entry.first)
      .kv("publisher_identity", sidecar.publisher_identity)
      .kv("reason", "token_expiring")
      .kv("pid", static_cast<int>(sidecar.pid))
      .info();

    tryRestartSidecar(entry.first, sidecar, "token_expiring");
  }
}

void VideoSidecarSupervisor::maintainSidecars()
{
  reapExitedSidecars();
  restartUnhealthy();
  restartExpiring();
}

}  // namespace livekit_ros2_bridge
