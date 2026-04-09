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

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "video_config.hpp"

namespace livekit_ros2_bridge
{

// Builds the bridge-owned sidecar argv using gstreamer-publisher and a resolved launch spec.
std::vector<std::string> buildGstreamerSidecarCommand(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token);

using SidecarCommandBuilder = std::function<std::vector<std::string>(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token)>;
using PublisherHealthCheck = std::function<bool(const std::string & publisher_identity)>;

// Owns the managed video sidecar child process for each sidecar_key.
// Callers drive maintenance by invoking reap/restart methods; the supervisor does not poll in the background.
class VideoSidecarSupervisor final
{
public:
  struct Config
  {
    std::string livekit_url;
    std::string livekit_room;
    std::string api_key;
    std::string api_secret;
    std::chrono::seconds token_ttl{3600};
    // restartExpiring() clamps this to at most half of token_ttl to avoid immediate refresh churn.
    std::chrono::seconds token_refresh_margin{300};
    // restartUnhealthy() ignores missing publications until a fresh sidecar has had time to join the room.
    std::chrono::milliseconds health_check_startup_grace{std::chrono::seconds(5)};
    // Consecutive failed health checks required before the managed sidecar is restarted.
    std::size_t unhealthy_restart_threshold{5};
    // Prefix for the publisher identity owned by each managed sidecar.
    std::string bridge_identity;
  };

  VideoSidecarSupervisor(
    Config config, SidecarCommandBuilder build_sidecar_command, PublisherHealthCheck is_publisher_healthy = {});
  ~VideoSidecarSupervisor();

  // Returns the stable publisher identity for spec.sidecar_key and ensures its managed child is running.
  std::string ensureSidecar(const SidecarLaunchSpec & spec);

  void stopSidecar(const std::string & sidecar_key);
  bool isSidecarRunning(const std::string & sidecar_key) const;
  void reapExitedSidecars();
  void restartUnhealthy();
  void restartExpiring();
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  struct SidecarRecord
  {
    SidecarLaunchSpec spec;
    // Stable LiveKit publisher identity currently assigned to this sidecar_key.
    std::string publisher_identity;
    pid_t pid = -1;
    // Expiration time for the token minted for the current child process.
    std::chrono::system_clock::time_point token_expires_at;
    SteadyClock::time_point spawned_at;
    // Cleared on spawn, stop, and healthy checks; counts consecutive failed health probes otherwise.
    std::size_t consecutive_unhealthy_checks = 0;
  };

  using SidecarMap = std::unordered_map<std::string, SidecarRecord>;

  struct PreparedSidecarLaunch
  {
    std::string publisher_identity;
    std::vector<std::string> argv;
    // Token lifetime for the replacement child prepared before any running child is killed.
    std::chrono::system_clock::time_point token_expires_at;
  };

  static std::string derivePublisherIdentity(const std::string & bridge_identity, const SidecarLaunchSpec & spec);
  static std::string keyToSlug(const std::string & key);

  std::pair<SidecarMap::iterator, bool> ensureSidecarRecord(const SidecarLaunchSpec & spec);
  void restartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar);
  void tryRestartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar, const char * reason);
  PreparedSidecarLaunch prepareSidecarLaunch(const SidecarRecord & sidecar) const;
  void spawnPreparedSidecar(const std::string & sidecar_key, SidecarRecord & sidecar, PreparedSidecarLaunch launch);
  void killSidecar(const std::string & sidecar_key, SidecarRecord & sidecar);

  Config config_;
  SidecarCommandBuilder build_sidecar_command_;
  PublisherHealthCheck is_publisher_healthy_;
  std::atomic<bool> is_shutdown_{false};
  // Managed children keyed by the canonical SidecarLaunchSpec::sidecar_key.
  SidecarMap sidecars_;
};

}  // namespace livekit_ros2_bridge
